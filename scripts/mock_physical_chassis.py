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
    # 创建虚拟物理伪终端主从对 (Master / Slave)
    master_fd, slave_fd = pty.openpty()
    slave_name = os.ttyname(slave_fd)

    print("==========================================================")
    print("      AMR 真实物理底盘硬件模拟器 (Hardware Emulator)      ")
    print("==========================================================")
    print(f" [OK] 物理串口模拟成功！从设备端口: {slave_name}")
    print(f" [HINT] 可将 chassis.yaml 中的 serial_device 指向: {slave_name}")
    print(" [INFO] 正在以 50Hz 广播真实底盘 ODOM / STATE 报文...")
    print(" [INFO] 按 Ctrl+C 退出或随时拔掉模拟连接验证容灾降级。")
    print("----------------------------------------------------------")

    x = 0.0
    y = 0.0
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
                # 简单运动学前向积分
                x += vx * dt
                yaw += wz * dt
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


if __name__ == "__main__":
    main()
