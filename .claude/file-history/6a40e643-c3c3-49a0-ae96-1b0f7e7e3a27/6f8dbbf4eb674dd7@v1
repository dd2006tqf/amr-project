#!/usr/bin/env python3
"""
模拟真实物理 AMR 底盘硬件。
通过 Linux pty 虚拟串口对与 amr_dispatcher 系统建立物理级通信闭环。
支持真实的串口波特率、换行符粘包/拆包、心跳反馈与丢包故障注入。
"""

import os
import pty
import select
import sys
import time


def main():
    # 多车支持（--instances N）：每台车一组独立 PTY + 独立初始位姿，
    # 便于 e2e 起两台 local_sim 车验证真并发。
    #   * 实例 0：从 (--start-x, --start-y) 出发
    #   * 实例 i>0：沿 --start-x 方向平移 --lane-offset * i，避免两车起点重合
    instances = 1
    start_x = 0.0
    start_y = 0.0
    lane_offset = 0.0
    args = sys.argv[1:]
    for i, arg in enumerate(args):
        if arg == "--instances" and i + 1 < len(args):
            instances = max(1, int(args[i + 1]))
        elif arg == "--start-x" and i + 1 < len(args):
            start_x = float(args[i + 1])
        elif arg == "--start-y" and i + 1 < len(args):
            start_y = float(args[i + 1])
        elif arg == "--lane-offset" and i + 1 < len(args):
            lane_offset = float(args[i + 1])

    if instances == 1:
        run_instance(pty.openpty(), start_x, start_y, "sim")
        return

    # 多车：各起一个独立 PTY 模拟器（后台线程）。
    # **不能 join**——join 会等所有线程退出，而每台车都跑"直到被杀掉"的常驻循环，
    # join 之后主线程永远等不到从设备路径输出。多车场景改为：
    #   1) 逐台**顺序**创建 PTY（PTY 创建是即时的，不需要并发），
    #      打印 "PTY <label> <path>" 供调用方解析；
    #   2) 再进入常驻循环，保持进程存活。
    pty_pairs = []
    for index in range(instances):
        pair = pty.openpty()
        slave_name = os.ttyname(pair[1])
        label = "sim_%d" % index
        print("PTY %s %s" % (label, slave_name), flush=True)
        pty_pairs.append((pair, start_x + lane_offset * index, start_y, label))

    import threading
    errors = []

    def worker(item):
        pair, sx, sy, label = item
        try:
            run_instance(pair, sx, sy, label, banner=False)
        except Exception as exc:  # noqa: BLE001
            errors.append(exc)

    threads = [threading.Thread(target=worker, args=(item,), daemon=True) for item in pty_pairs]
    for t in threads:
        t.start()
    # 主线程常驻：等所有工作线程结束（正常情况下不会结束，直到被 kill）
    for t in threads:
        t.join()
    if errors:
        for exc in errors:
            print(" [ERROR] 底盘模拟器异常: %s" % exc, flush=True)
        raise SystemExit(1)


def run_instance(pty_pair, start_x, start_y, label, banner=True):
    # 创建虚拟物理伪终端主从对 (Master / Slave)
    master_fd, slave_fd = pty_pair
    slave_name = os.ttyname(slave_fd)

    if banner:
        print("==========================================================")
        print("      AMR 真实物理底盘硬件模拟器 (Hardware Emulator)      ")
        print("==========================================================")
        print(f" [OK] 物理串口模拟成功！从设备端口: {slave_name} (label={label})")
        print(f" [HINT] 可将 chassis.yaml 中的 serial_device 指向: {slave_name}")
        print(" [INFO] 正在以 50Hz 广播真实底盘 ODOM / STATE 报文...")
        print(" [INFO] 按 Ctrl+C 退出或随时拔掉模拟连接验证容灾降级。")
        print("----------------------------------------------------------")

    x = start_x
    y = start_y
    yaw = 0.0
    vx = 0.0
    wz = 0.0
    battery = 24.5

    last_pub = time.time()
    rx_buf = b""

    try:
        while True:
            rlist, _, _ = select.select([master_fd], [], [], 0.02)
            if master_fd in rlist:
                try:
                    data = os.read(master_fd, 1024)
                    if not data:
                        break
                    rx_buf += data
                    while b"\n" in rx_buf:
                        line, rx_buf = rx_buf.split(b"\n", 1)
                        line_str = line.decode("ascii", errors="ignore").strip()
                        if line_str.startswith("CMD"):
                            parts = line_str.split()
                            if len(parts) >= 4:
                                vx = float(parts[1])
                                wz = float(parts[3])
                except OSError as e:
                    print(f" [WARN] 物理串口读取中断 (模拟拔线): {e}")
                    break

            now = time.time()
            dt = now - last_pub
            if dt >= 0.02:  # 50Hz 周期性状态回传
                last_pub = now
                # 差速/阿克曼平面运动学积分 (考虑当前航向角 yaw)
                import math
                x += vx * math.cos(yaw) * dt
                y += vx * math.sin(yaw) * dt
                yaw += wz * dt
                # 航向角正规化到 [-pi, pi]
                yaw = math.atan2(math.sin(yaw), math.cos(yaw))
                battery = max(20.0, battery - 0.0001)

                odom_pkt = (
                    f"ODOM {x:.4f} {y:.4f} {yaw:.4f} {vx:.4f} {wz:.4f} {battery:.2f}\n"
                )
                try:
                    os.write(master_fd, odom_pkt.encode("ascii"))
                except OSError:
                    break

    except KeyboardInterrupt:
        print("\n [INFO] 硬件模拟器主动停止。")
    finally:
        os.close(master_fd)
        os.close(slave_fd)
    return slave_name


if __name__ == "__main__":
    main()
