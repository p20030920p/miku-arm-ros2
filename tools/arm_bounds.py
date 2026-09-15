#!/usr/bin/env python3
"""按 URDF 算正运动学，给出机械臂当前位形的包围盒与相机取景参数。

只处理本仓库的 URDF（revolute/fixed 关节，origin 带 xyz/rpy，axis 固定），
够用就行，不引入 KDL 依赖。

    python3 tools/arm_bounds.py                 # 零位
    python3 tools/arm_bounds.py --gif           # 打印 RViz 相机参数
"""
import argparse
import math
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
URDF = os.path.join(ROOT, "src/miku_dummy/urdf/miku_dummy.urdf")


def mat_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)]
            for i in range(4)]


def rpy_to_mat(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def origin_mat(xyz, rpy):
    m = [[0.0] * 4 for _ in range(4)]
    rot = rpy_to_mat(*rpy)
    for i in range(3):
        for j in range(3):
            m[i][j] = rot[i][j]
        m[i][3] = xyz[i]
    m[3][3] = 1.0
    return m


def parse_urdf(path):
    text = re.sub(r"\s+", " ", open(path, encoding="utf-8").read())
    joints = []
    for m in re.finditer(r'<joint name="([^"]+)" type="([^"]+)">(.*?)</joint>', text):
        name, jtype, body = m.groups()
        o = re.search(r'<origin xyz="([^"]+)"(?:\s+rpy="([^"]+)")?\s*/?>', body)
        xyz = [float(v) for v in o.group(1).split()] if o else [0.0, 0.0, 0.0]
        rpy = [float(v) for v in o.group(2).split()] if (o and o.group(2)) else [0.0, 0.0, 0.0]
        p = re.search(r'<parent link="([^"]+)"', body)
        c = re.search(r'<child link="([^"]+)"', body)
        ax = re.search(r'<axis xyz="([^"]+)"', body)
        axis = [float(v) for v in ax.group(1).split()] if ax else [0.0, 0.0, 1.0]
        joints.append(dict(name=name, type=jtype, xyz=xyz, rpy=rpy,
                           parent=p.group(1), child=c.group(1), axis=axis))
    if not joints:
        raise SystemExit(f"没从 {path} 解析出关节")
    children = {j["child"] for j in joints}
    roots = [j["parent"] for j in joints if j["parent"] not in children]
    return joints, roots[0]


def axis_angle(axis, angle):
    n = math.sqrt(sum(v * v for v in axis)) or 1.0
    x, y, z = (v / n for v in axis)
    c, s, C = math.cos(angle), math.sin(angle), 1 - math.cos(angle)
    return [[x * x * C + c, x * y * C - z * s, x * z * C + y * s, 0],
            [y * x * C + z * s, y * y * C + c, y * z * C - x * s, 0],
            [z * x * C - y * s, z * y * C + x * s, z * z * C + c, 0],
            [0, 0, 0, 1]]


def fk(joints, root, q):
    """返回 {link: 4x4 矩阵}，以 URDF 根连杆为原点。"""
    eye = [[1.0 if i == j else 0.0 for j in range(4)] for i in range(4)]
    pose = {root: eye}
    frames = {root: eye}
    remaining = list(joints)
    while remaining:
        progressed = False
        for j in list(remaining):
            if j["parent"] in pose:
                m = mat_mul(pose[j["parent"]], origin_mat(j["xyz"], j["rpy"]))
                if j["type"] != "fixed":
                    m = mat_mul(m, axis_angle(j["axis"], q.get(j["name"], 0.0)))
                pose[j["child"]] = m
                frames[j["child"]] = m
                remaining.remove(j)
                progressed = True
        if not progressed:
            raise SystemExit(f"URDF 关节链断开，剩余：{[j['name'] for j in remaining]}")
    return frames


def bbox(frames):
    pts = [(m[0][3], m[1][3], m[2][3]) for m in frames.values()]
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    zs = [p[2] for p in pts]
    return (min(xs), max(xs)), (min(ys), max(ys)), (min(zs), max(zs))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gif", action="store_true", help="输出 RViz 相机参数")
    ap.add_argument("--quality", type=float, default=1.0, help="视野余量系数")
    args = ap.parse_args()

    joints, root = parse_urdf(URDF)
    print(f"URDF 根连杆：{root}")
    frames = fk(joints, root, {})
    (x0, x1), (y0, y1), (z0, z1) = bbox(frames)
    print("零位各连杆位置（URDF 根坐标系）：")
    for name, m in frames.items():
        print(f"  {name:10s} ({m[0][3]:+.3f}, {m[1][3]:+.3f}, {m[2][3]:+.3f})")
    print(f"\n包围盒 X[{x0:+.3f},{x1:+.3f}] Y[{y0:+.3f},{y1:+.3f}] Z[{z0:+.3f},{z1:+.3f}]")

    # 取景：以包围盒中心为目标，按最长边和视场角反推距离
    cx, cy, cz = (x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2
    span = max(x1 - x0, y1 - y0, z1 - z0)
    # 竖直视口窄，按竖直方向留量
    fov = math.radians(60.0)
    dist = args.quality * span / (2 * math.tan(fov / 2))
    print(f"\n目标点 ({cx:+.3f}, {cy:+.3f}, {cz:+.3f})  最长边 {span:.3f} m")
    if args.gif:
        print(f"建议 RViz：Distance {dist:.2f}  Yaw {math.pi/2:.4f}  Pitch 0.12  "
              f"Focal Point ({cx:.3f}; {cy:.3f}; {cz:.3f})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
