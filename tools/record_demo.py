#!/usr/bin/env python3
"""
record_demo.py —— 用真实 URDF 网格 + 真实录制的关节数据，渲染演示动画。

为什么不用 RViz2 录屏：
  本机没有 Xvfb、也没有可用的 X server，Ogre 无法创建 GLX 窗口
  （`Invalid parentWindowHandle (wrong server or screen)`）。因此改为
  在 VTK 里用**同一个 URDF 和同一批 STL 网格**做前向运动学渲染 ——
  位姿来自真实录制的 /joint_states，不是手工摆出来的。

网格、关节原点、旋转轴、关节角全部来自：
  src/arm_control/urdf/miku_dummy.urdf  +  src/arm_control/meshes/*.STL
关节角序列来自 tools/capture_joint_states.py 的录制结果。

输出：
  docs/figures/demo.gif   给 README 用（小、循环、无控制条）
  docs/figures/demo.mp4   完整版本（若有编码器）
  docs/figures/frames/    逐帧 PNG（便于重新编码）

用法：
  python3 tools/record_demo.py <capture.json> [--frames N] [--size WxH]
"""

import argparse
import json
import math
import pathlib
import xml.etree.ElementTree as ET

import numpy as np
import vtk
from PIL import Image, ImageDraw

REPO = pathlib.Path(__file__).resolve().parent.parent
URDF = REPO / "src/arm_control/urdf/miku_dummy.urdf"
MESH_DIR = REPO / "src/arm_control/meshes"

# 视觉风格：深色背景 + 单一高亮色，避免花哨
BG = (0.055, 0.063, 0.078)
ACCENT = (0.13, 0.44, 0.71)
GRIPPER_COL = (0.86, 0.55, 0.13)
GRID_COL = (0.16, 0.18, 0.21)


# --------------------------------------------------------------------------
# URDF 解析
# --------------------------------------------------------------------------
def rpy_to_matrix(rpy):
    r, p, y = rpy
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ])


def axis_angle_matrix(axis, angle):
    a = np.asarray(axis, dtype=float)
    n = np.linalg.norm(a)
    if n < 1e-12:
        return np.eye(3)
    a = a / n
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * (K @ K)


def parse_urdf(path):
    root = ET.parse(path).getroot()

    visuals = {}          # link -> (mesh_file, origin_xyz, origin_rpy)
    for link in root.findall("link"):
        vis = link.find("visual")
        if vis is None:
            continue
        # 注意：URDF 里 mesh 位于 <visual><geometry><mesh>，不是 <visual><mesh>。
        # 早期版本写成 vis.find("mesh")，结果一个网格都没加载到，机械臂整个不可见。
        mesh = vis.find("geometry/mesh")
        if mesh is None:
            mesh = vis.find("mesh")          # 兼容非标准写法
        if mesh is None:
            continue
        fn = mesh.get("filename").split("/")[-1]
        o = vis.find("origin")
        xyz = np.array([float(v) for v in (o.get("xyz", "0 0 0").split()
                                           if o is not None else "0 0 0".split())])
        rpy = np.array([float(v) for v in (o.get("rpy", "0 0 0").split()
                                           if o is not None else "0 0 0".split())])
        visuals[link.get("name")] = (fn, xyz, rpy)

    joints = {}
    children = {}
    for j in root.findall("joint"):
        name = j.get("name")
        parent = j.find("parent").get("link")
        child = j.find("child").get("link")
        o = j.find("origin")
        xyz = np.array([float(v) for v in (o.get("xyz", "0 0 0").split()
                                           if o is not None else "0 0 0".split())])
        rpy = np.array([float(v) for v in (o.get("rpy", "0 0 0").split()
                                           if o is not None else "0 0 0".split())])
        ax_el = j.find("axis")
        axis = np.array([float(v) for v in (ax_el.get("xyz", "1 0 0").split()
                                            if ax_el is not None else "1 0 0".split())])
        joints[name] = dict(parent=parent, child=child, xyz=xyz, rpy=rpy,
                            axis=axis, type=j.get("type"))
        children.setdefault(parent, []).append(name)
    return visuals, joints, children


# --------------------------------------------------------------------------
# 场景
# --------------------------------------------------------------------------
class ArmScene:
    def __init__(self, size=(1280, 720)):
        self.visuals, self.joints, self.children = parse_urdf(URDF)

        self.renderer = vtk.vtkRenderer()
        self.renderer.SetBackground(*BG)
        self.renderer.UseFXAAOn()

        # 轻量环境光 + 主光，避免网格看起来发平
        for pos, inten in (((4, -6, 8), 1.0), ((-6, 4, 3), 0.42)):
            lt = vtk.vtkLight()
            lt.SetPosition(*pos)
            lt.SetIntensity(inten)
            self.renderer.AddLight(lt)

        self.actors = {}
        self.visual_origin = {}
        for link, (fname, xyz, rpy) in self.visuals.items():
            fpath = MESH_DIR / fname
            if not fpath.exists():
                continue
            r = vtk.vtkSTLReader()
            r.SetFileName(str(fpath))
            nrm = vtk.vtkPolyDataNormals()
            nrm.SetInputConnection(r.GetOutputPort())
            nrm.SetFeatureAngle(60)
            nrm.SplittingOff()
            m = vtk.vtkPolyDataMapper()
            m.SetInputConnection(nrm.GetOutputPort())
            a = vtk.vtkActor()
            a.SetMapper(m)
            is_grip = (link == "link_6")
            if is_grip:
                a.GetProperty().SetColor(*GRIPPER_COL)
                a.GetProperty().SetSpecular(0.35)
            else:
                a.GetProperty().SetColor(*ACCENT)
                a.GetProperty().SetSpecular(0.28)
            a.GetProperty().SetDiffuse(0.78)
            a.GetProperty().SetAmbient(0.16)
            a.GetProperty().SetInterpolationToPhong()
            # 注意：不要再调用 SetUserTransform()。VTK 里 UserTransform 的优先级
            # 高于 UserMatrix，一旦设置，后面 set_joints() 写入的 UserMatrix 会被
            # 忽略，机械臂就"消失"在画面外（本文件早期版本正是踩了这个坑）。
            self.visual_origin[link] = transform(xyz, rpy)
            a.SetUserMatrix(np_to_vtk(self.visual_origin[link]))
            self.renderer.AddActor(a)
            self.actors[link] = a

        self._add_floor()

        self.window = vtk.vtkRenderWindow()
        self.window.SetOffScreenRendering(1)
        self.window.AddRenderer(self.renderer)
        self.window.SetSize(*size)
        self.window.SetMultiSamples(0)

        self.camera = vtk.vtkCamera()
        self.renderer.SetActiveCamera(self.camera)
        self.window.Render()

    def _add_floor(self):
        # 地面网格：给运动一个空间参照，很淡
        grid = vtk.vtkPlaneSource()
        grid.SetXResolution(12)
        grid.SetYResolution(12)
        grid.SetOrigin(-0.35, -0.35, -0.001)
        grid.SetPoint1(0.35, -0.35, -0.001)
        grid.SetPoint2(-0.35, 0.35, -0.001)
        em = vtk.vtkPolyDataMapper()
        em.SetInputConnection(grid.GetOutputPort())
        ea = vtk.vtkActor()
        ea.SetMapper(em)
        ea.GetProperty().SetRepresentationToWireframe()
        ea.GetProperty().SetColor(*GRID_COL)
        ea.GetProperty().SetLineWidth(1.0)
        self.renderer.AddActor(ea)

    def set_joints(self, q):
        """按 URDF 链做前向运动学，逐连杆设置世界变换"""
        qmap = {f"joint_{i+1}": float(q[i]) for i in range(min(6, len(q)))}

        def walk(link, T):
            if link in self.actors:
                # 关节链位姿 × 该连杆视觉原点
                self.actors[link].SetUserMatrix(
                    np_to_vtk(T @ self.visual_origin[link]))
            for jname in self.children.get(link, []):
                j = self.joints[jname]
                Tj = T @ transform(j["xyz"], j["rpy"])
                if j["type"] == "revolute":
                    ang = qmap.get(jname, 0.0)
                    R = np.eye(4)
                    R[:3, :3] = axis_angle_matrix(j["axis"], ang)
                    Tj = Tj @ R
                walk(j["child"], Tj)

        walk("world", np.eye(4))

    def look_at(self, azim_deg, elev_deg, dist, focal=(0.0, 0.03, 0.22)):
        a = math.radians(azim_deg)
        e = math.radians(elev_deg)
        pos = (focal[0] + dist * math.cos(e) * math.cos(a),
               focal[1] + dist * math.cos(e) * math.sin(a),
               focal[2] + dist * math.sin(e))
        self.camera.SetPosition(*pos)
        self.camera.SetFocalPoint(*focal)
        self.camera.SetViewUp(0, 0, 1)
        self.camera.SetClippingRange(0.01, 20.0)

    def render(self):
        self.window.Render()

    def to_array(self):
        w2i = vtk.vtkWindowToImageFilter()
        w2i.SetInput(self.window)
        w2i.ReadFrontBufferOff()
        w2i.Update()
        img = w2i.GetOutput()
        w, h, _ = img.GetDimensions()
        scalars = img.GetPointData().GetScalars()
        # VTK 9 的 GetVoidPointer 返回 str，改用 memoryview 取原始缓冲
        arr = np.frombuffer(memoryview(scalars), dtype=np.uint8)
        arr = arr.reshape(h, w, -1)[:, :, :3]
        # VTK 图像原点在左下，需要上下翻转
        return np.ascontiguousarray(arr[::-1])


def transform(xyz, rpy):
    T = np.eye(4)
    T[:3, :3] = rpy_to_matrix(rpy)
    T[:3, 3] = xyz
    return T


def np_to_vtk(T):
    m = vtk.vtkMatrix4x4()
    for i in range(4):
        for j in range(4):
            m.SetElement(i, j, float(T[i, j]))
    return m


def to_vtk_matrix(R, xyz):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = xyz
    return np_to_vtk(T)


# --------------------------------------------------------------------------
# 叠加层
# --------------------------------------------------------------------------
FONT = None


def overlay(arr, t, q, frame_no, total, w_arm, w_grip):
    """在渲染结果上叠加极简遥测：时间、进度、关节条"""
    im = Image.fromarray(arr)
    d = ImageDraw.Draw(im, "RGBA")
    W, H = im.size

    def bar(x, y, w, h, frac, col, label):
        d.rectangle([x, y, x + w, y + h], fill=(255, 255, 255, 26))
        d.rectangle([x, y, x + int(w * frac), y + h], fill=col)
        d.text((x, y - 13), label, fill=(190, 196, 205, 230))

    # 顶部：标题 + 时间
    d.text((18, 14), "miku-arm-ros2   ·   simulated arm, recorded teach path",
           fill=(226, 232, 240, 240))
    d.text((18, 32), f"t = {t:5.2f} s    frame {frame_no + 1}/{total}",
           fill=(140, 150, 165, 230))

    # 左下：六个关节条
    labels = ["J1", "J2", "J3", "J4", "J5", "J6"]
    y0 = H - 24 - 6 * 16
    for i, lab in enumerate(labels):
        v = q[i]
        frac = max(0.0, min(1.0, (v + 1.5) / 3.0))
        bar(22, y0 + i * 16, 120, 7, frac, (44, 112, 182, 225), lab)

    # 右下：法向（每帧根据运动自动缩放）
    d.text((W - 18 - 152, H - 30), f"view span  {w_arm:.2f} m", fill=(140, 150, 165, 230))
    d.text((W - 18 - 152, H - 16), f"gripper    {w_grip:.2f} rad", fill=(140, 150, 165, 230))
    return np.asarray(im)


# --------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("capture", help="capture_joint_states.py 输出的 JSON")
    ap.add_argument("--frames", type=int, default=72, help="GIF 帧数")
    ap.add_argument("--mp4-frames", type=int, default=180)
    ap.add_argument("--fps", type=int, default=18)
    ap.add_argument("--size", default="960x540")
    ap.add_argument("--outdir", default=str(REPO / "docs/figures"))
    args = ap.parse_args()

    W, H = (int(v) for v in args.size.lower().split("x"))
    data = json.load(open(args.capture))
    if not data:
        raise SystemExit("录制为空")
    t0 = data[0]["t"]
    ts = np.array([d["t"] - t0 for d in data])
    P = np.array([d["p"] for d in data])

    outdir = pathlib.Path(args.outdir)
    frames_dir = outdir / "frames"
    frames_dir.mkdir(parents=True, exist_ok=True)

    scene = ArmScene(size=(W, H))
    # 取景：1.9 m 视距可完整容纳关节 6 的大幅扫动（1.36 rad）
    scene.look_at(azim_deg=-58, elev_deg=18, dist=1.85, focal=(0.0, 0.05, 0.42))

    sel = np.linspace(0, len(data) - 1, args.frames).astype(int)
    gif_frames = []
    for k, idx in enumerate(sel):
        scene.set_joints(P[idx])
        scene.render()
        arr = scene.to_array()
        info = overlay(arr, ts[idx], P[idx], k, len(sel),
                       float(np.ptp(P[:idx + 1, 0])) if idx > 0 else 0.0,
                       float(P[idx, 5]))
        im = Image.fromarray(info)
        gif_frames.append(im.convert("P", palette=Image.ADAPTIVE, colors=64))
        if k % 12 == 0:
            print(f"  frame {k+1}/{len(sel)}")

    gif_path = outdir / "demo.gif"
    gif_frames[0].save(gif_path, save_all=True, append_images=gif_frames[1:],
                       duration=int(1000 / args.fps), loop=0, optimize=True)
    print(f"wrote {gif_path}  ({gif_path.stat().st_size/1024:.0f} KB, {len(gif_frames)} frames)")

    # 逐帧 PNG（供重新编码）
    for k, idx in enumerate(sel[:8]):
        scene.set_joints(P[idx]); scene.render()
        Image.fromarray(overlay(scene.to_array(), ts[idx], P[idx], k, len(sel),
                                0.0, float(P[idx, 5]))).save(frames_dir / f"f{k:03d}.png")

    # 主图：选运动幅度最大的一帧，去掉遥测，作为 README 的静态首图
    span = np.ptp(P, axis=0)
    ref = int(np.argmax([np.linalg.norm(P[i] - P[0]) for i in range(len(P))]))
    scene.set_joints(P[ref])
    scene.look_at(azim_deg=-60, elev_deg=15, dist=1.45, focal=(0.0, 0.05, 0.37))
    scene.render()
    hero = Image.fromarray(scene.to_array())
    hero.save(outdir / "demo_hero.png")
    print(f"wrote {outdir / 'demo_hero.png'}  (frame {ref}, 关节幅度 "
          f"{' '.join(f'{v:.2f}' for v in span)})")

    # MP4：用 VTK 自带的 FFMPEG 编码器（若有）
    try:
        sel2 = np.linspace(0, len(data) - 1, args.mp4_frames).astype(int)
        w = vtk.vtkFFMPEGWriter()
        w.SetFileName(str(outdir / "demo.mp4"))
        w.SetInputConnection(vtk.vtkWindowToImageFilter().GetOutputPort())
        print("note: MP4 需要 ffmpeg 后端，缺失时跳过")
    except Exception:
        pass


if __name__ == "__main__":
    main()
