#!/usr/bin/env python3
# ============================================================================
# virtual_motor_board.py —— 虚拟达妙电机驱动板（串口协议级仿真）
#
# 为什么需要它：
#   本工程最无法在无实机条件下验证的部分，是 hardware 节点与下位机之间的
#   **二进制串口协议**。本脚本忠实实现该协议的另一端，配合 socat 创建的虚拟
#   串口（PTY），就可以让**真正的 hardware 可执行文件**跑起来，从而验证：
#     - 50 字节下行指令帧的解析（帧头、字段偏移、×1000 定点换算）
#     - 46 字节上行反馈帧的生成与 hardware 端的解析
#     - 位置模式的电机响应、力矩反馈
#     - 夹爪的夹取/失败状态机（配合 claw_test / claw_arm_test）
#     - 串口断开/重连逻辑（可以中途杀掉本进程再拉起）
#
# 协议（来自 hardware.cpp，逐字节对齐）：
#   下行 TX  50 字节： 86 C1 | status mode | 电机1..7 各 (pos,vel,tor) 各 2 字节大端
#                      | kp kd（偏移 46..49）
#                       电机 i (1-based) 起始偏移 = 4 + (i-1)*6；电机7 在 40
#   上行 RX  46 字节： 86 C2 | status mode | 电机1..7 各 (pos,vel,tor) 大端
#                       电机 i 起始偏移 = 4 + (i-1)*6；电机7 在 40
#   所有物理量 ×1000 后以 int16 传输（位置 rad / 速度 rad·s⁻¹ / 力矩 N·m）
#
# 用法：
#   # 1) 建虚拟串口对（一个给 hardware，一个给本脚本）
#   socat -d -d pty,raw,echo=0,link=/tmp/ttyARM0 pty,raw,echo=0,link=/tmp/ttyARM1
#   # 2) 启动虚拟驱动板（连到 ttyARM1，hardware 连 ttyARM0）
#   python3 virtual_motor_board.py --port /tmp/ttyARM1
#   # 3) 启动 hardware 并指向虚拟串口
#   ros2 run hardware hardware --ros-args -p serial_port:=/tmp/ttyARM0
# ============================================================================

import argparse
import math
import os
import struct
import sys
import time

TX_HEADER = (0x86, 0xC1)
RX_HEADER = (0x86, 0xC2)
TX_LEN = 50
RX_LEN = 46

# 与 hardware.cpp 中 GRIPPER_* 常量一致的夹爪物理限位（单位：弧度）
GRIPPER_POS_UPPER_RAD = 0.0
GRIPPER_POS_LOWER_RAD = -165.0 * math.pi / 180.0
GRIPPER_MAX_TORQUE = 1.50


def i16(v):
    """物理量 -> int16（×1000，饱和截断），与硬件端的定点约定一致"""
    raw = int(round(v * 1000.0))
    if raw > 32767:
        raw = 32767
    elif raw < -32768:
        raw = -32768
    return raw & 0xFFFF


def be16(buf, off):
    """从大端字节流取有符号 int16"""
    return struct.unpack_from('>h', buf, off)[0]


class MotorPlant:
    """
    单台电机的简化模型。

    位置模式(mode=2)：一阶趋近目标位置，速度取"实际位移/时间"的反推值
                       （与真实电机的位置环行为定性一致，且便于轨迹复现比对）。
    MIT 模式(mode=1)：位置刚度 kp 与力矩前馈 tor 共同作用；
                       这里用 tau = kp*(pos_cmd - pos) + tor 的一阶模型，
                       并叠加重力负载由上层(arm_control)通过 tor 前馈补偿——
                       也就是"如果上层的重力补偿是对的，机械臂就静止不动"。
    """

    def __init__(self, index):
        self.index = index
        self.pos = 0.0
        self.vel = 0.0
        self.tor = 0.0
        self.cmd_pos = 0.0
        self.cmd_vel = 0.0
        self.cmd_tor = 0.0

    # 关节 2、3 承受主要重力负载（简化模型）。
    # 关键点：两种模式下重力都必须起作用，否则无法看出重力补偿的价值：
    #   - 位置模式：位置环刚度有限，负载造成**稳态下垂** g/kp
    #   - MIT 模式：kp 同样有限，且没有力矩前馈时下垂更明显
    GRAVITY = {2: 0.45, 3: 0.10}

    def step(self, mode, kp, kd, dt, apply_gravity):
        g = self.GRAVITY.get(self.index, 0.0) if apply_gravity else 0.0

        if mode == 2:
            # ---- 速度位置模式：朝目标运动，速度上限 |cmd_vel| ----
            # 用 KP_POS 充当位置环刚度，产生可观测的稳态下垂（真实伺服同理）
            KP_POS = 8.0
            vmax = abs(self.cmd_vel)
            if vmax < 1e-6:
                vmax = 0.5
            # 位置环的目标"等效位置"：受负载影响偏 g/kp
            eff_target = self.cmd_pos - g / KP_POS
            err = eff_target - self.pos
            step = max(-vmax * dt, min(vmax * dt, err))
            self.pos += step
            self.vel = step / dt if dt > 0 else 0.0
            self.tor = g   # 位置模式下电机为顶住负载输出的力矩（回读可见）
        else:
            # ---- MIT 模式：kp 拉向目标 + 力矩前馈 ----
            err = self.cmd_pos - self.pos
            tau = kp * err + self.cmd_tor - g
            # 一阶惯性：kd 作为阻尼
            acc = tau - kd * self.vel * 10.0
            self.vel += acc * dt * 20.0
            self.pos += self.vel * dt
            self.tor = self.cmd_tor

    def clamp(self, lo=-3.2, hi=3.2):
        if self.pos < lo:
            self.pos = lo
        elif self.pos > hi:
            self.pos = hi


class VirtualMotorBoard:
    def __init__(self, port, baudrate, quiet=False, grip_object=True,
                 gravity=True):
        self.port = port
        self.baudrate = baudrate
        self.quiet = quiet
        self.grip_object = grip_object
        self.gravity = gravity

        # 电机 1..6 + 夹爪 7
        self.motors = [MotorPlant(i) for i in range(1, 8)]

        self.status = 0
        self.mode = 1
        self.kp = 0.0
        self.kd = 0.0

        self.rx_buf = bytearray()
        self.frames_rx = 0
        self.frames_tx = 0
        self.last_stats = time.time()

    # ------------------------------------------------------------------
    def _parse_tx(self, frame):
        """解析 50 字节下行指令帧 -> 更新各电机目标值"""
        if frame[0] != TX_HEADER[0] or frame[1] != TX_HEADER[1]:
            return False
        self.status = frame[2]
        self.mode = frame[3]
        for i in range(7):
            off = 4 + i * 6
            self.motors[i].cmd_pos = be16(frame, off) / 1000.0
            self.motors[i].cmd_vel = be16(frame, off + 2) / 1000.0
            self.motors[i].cmd_tor = be16(frame, off + 4) / 1000.0
        self.kp = be16(frame, 46) / 1000.0
        self.kd = be16(frame, 48) / 1000.0
        self.frames_rx += 1
        return True

    def _build_rx(self):
        """生成 46 字节上行反馈帧（位置/速度/力矩 ×1000 大端）"""
        buf = bytearray(RX_LEN)
        buf[0], buf[1] = RX_HEADER
        buf[2] = self.status & 0xFF
        buf[3] = self.mode & 0xFF
        for i, m in enumerate(self.motors):
            off = 4 + i * 6
            struct.pack_into('>H', buf, off, i16(m.pos))
            struct.pack_into('>H', buf, off + 2, i16(m.vel))
            struct.pack_into('>H', buf, off + 4, i16(m.tor))
        self.frames_tx += 1
        return bytes(buf)

    # ------------------------------------------------------------------
    def run(self, step_dt=0.01):
        fd = os.open(self.port, os.O_RDWR | os.O_NOCTTY)
        print(f"[虚拟驱动板] 已连接 {self.port}（波特率参数 {self.baudrate}，PTY 无实际波特率）",
              flush=True)
        next_step = time.monotonic()
        try:
            while True:
                # ---- 读（非阻塞）----
                try:
                    data = os.read(fd, 4096)
                except BlockingIOError:
                    data = b''
                except OSError as e:
                    print(f"[虚拟驱动板] 串口读失败：{e}", flush=True)
                    break
                if data:
                    self.rx_buf.extend(data)
                    # 按帧头切分并解析
                    while True:
                        idx = self.rx_buf.find(bytes(TX_HEADER))
                        if idx < 0:
                            if len(self.rx_buf) > 4096:
                                self.rx_buf.clear()
                            break
                        if len(self.rx_buf) - idx < TX_LEN:
                            if idx > 0:
                                del self.rx_buf[:idx]
                            break
                        frame = bytes(self.rx_buf[idx:idx + TX_LEN])
                        del self.rx_buf[:idx + TX_LEN]
                        self._parse_tx(frame)

                # ---- 按固定步长推进物理仿真 ----
                now = time.monotonic()
                while now >= next_step:
                    dt = step_dt
                    # 夹爪（电机7）：施加限位与"夹到物体"的力矩特征
                    for i, m in enumerate(self.motors):
                        m.step(self.mode, self.kp, self.kd, dt,
                               apply_gravity=self.gravity)
                        if i < 6:
                            m.clamp()
                        else:
                            # 夹爪限位 + 夹到物体时产生反力矩
                            if self.grip_object and m.pos <= -1.2:
                                # 夹到物体：位置卡住、力矩上升（供夹取判定）
                                m.pos = -1.2
                                m.vel = 0.0
                                m.tor = 0.6
                            else:
                                m.clamp(GRIPPER_POS_LOWER_RAD, GRIPPER_POS_UPPER_RAD)
                    next_step += dt

                # ---- 回发反馈帧 ----
                os.write(fd, self._build_rx())

                # ---- 统计输出 ----
                if not self.quiet and time.time() - self.last_stats > 5.0:
                    self.last_stats = time.time()
                    p = " ".join(f"{m.pos:+.3f}" for m in self.motors)
                    print(f"[虚拟驱动板] 收 {self.frames_rx} 帧 / 发 {self.frames_tx} 帧 "
                          f"| mode={self.mode} kp={self.kp:.2f} | pos=[{p}]", flush=True)

                # ---- 10ms 周期（与 hardware.cpp 主循环一致）----
                time.sleep(0.002)
        finally:
            os.close(fd)
            print(f"[虚拟驱动板] 退出。共收 {self.frames_rx} 帧，发 {self.frames_tx} 帧",
                  flush=True)


def main():
    ap = argparse.ArgumentParser(description="虚拟达妙电机驱动板（串口协议级仿真）")
    ap.add_argument('--port', required=True, help='虚拟串口设备，如 /tmp/ttyARM1')
    ap.add_argument('--baudrate', type=int, default=115200, help='仅记录用，PTY 无实际波特率')
    ap.add_argument('--quiet', action='store_true', help='不打印周期统计')
    ap.add_argument('--no-object', action='store_true',
                    help='夹爪里不放物体（用于测试"未夹到"分支）')
    ap.add_argument('--no-gravity', action='store_true',
                    help='关闭重力负载：用于纯协议保真度测试（位置应精确跟随指令）')
    args = ap.parse_args()

    try:
        VirtualMotorBoard(args.port, args.baudrate, args.quiet,
                          grip_object=not args.no_object,
                          gravity=not args.no_gravity).run()
    except KeyboardInterrupt:
        pass
    except FileNotFoundError:
        print(f"错误：虚拟串口 {args.port} 不存在。请先用 socat 创建串口对。", file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
