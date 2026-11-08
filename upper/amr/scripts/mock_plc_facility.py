#!/usr/bin/env python3
"""
模拟现场工业 PLC 设施（Modbus-TCP 服务端）。
用于 Phase 8 物理设施联锁集成测试 (P3):
- 支持 MBAP 报文解析 (Transaction ID, Protocol ID, Length, Unit ID)
- 支持功能码:
  - 0x01: Read Coils (读门线圈: 0=Open Limit Switch, 1=Closed Limit Switch)
  - 0x03: Read Holding Registers (读保持寄存器: 0=目标楼层, 1=当前实时楼层)
  - 0x05: Write Single Coil (写线圈: 0=触发开门, 1=触发关门)
  - 0x06: Write Single Register (写保持寄存器: 0=设定目标楼层)
- 具备物理机械动作仿真时延（开门需要 N 秒到位）与故障模拟（可注入卡死/无响应）。
"""

import argparse
import socket
import struct
import threading
import time
import sys

class MockPlcFacility:
    def __init__(self, host="127.0.0.1", port=15020, door_open_delay=1.0, fault_mode="none"):
        self.host = host
        self.port = port
        self.door_open_delay = door_open_delay
        self.fault_mode = fault_mode  # "none", "stuck_closed", "timeout"
        self.running = False

        # 状态变量
        # 门线圈: [coil 0 (open limit), coil 1 (closed limit)]
        # 默认门完全关闭: open=False, closed=True
        self.door_coils = [False, True]
        self.door_state = "closed" # "closed", "opening", "open", "closing"

        # 电梯寄存器: [reg 0 (target floor), reg 1 (current floor)]
        self.elevator_regs = [1, 1]

        self.lock = threading.Lock()
        self.server_sock = None

    def _simulate_door_motion(self, target_state):
        with self.lock:
            self.door_state = "opening" if target_state == "open" else "closing"
            # 运动过程中限位开关两边都为 0 (moving)
            self.door_coils = [False, False]

        if self.fault_mode == "stuck_closed" and target_state == "open":
            # 模拟门卡死故障：永远无法开到位，超时降级
            time.sleep(self.door_open_delay)
            with self.lock:
                self.door_state = "closed"
                self.door_coils = [False, True]
            return

        time.sleep(self.door_open_delay)
        with self.lock:
            if target_state == "open":
                self.door_state = "open"
                self.door_coils = [True, False]
            else:
                self.door_state = "closed"
                self.door_coils = [False, True]

    def _handle_client(self, client_sock):
        try:
            while self.running:
                # 读 MBAP 头部 7 字节
                header = client_sock.recv(7)
                if not header or len(header) < 7:
                    break
                tx_id, proto_id, length, unit_id = struct.unpack(">HHHB", header)
                if proto_id != 0:
                    continue
                pdu_len = length - 1
                pdu = b""
                while len(pdu) < pdu_len:
                    chunk = client_sock.recv(pdu_len - len(pdu))
                    if not chunk:
                        break
                    pdu += chunk

                if len(pdu) < pdu_len:
                    break

                if self.fault_mode == "timeout":
                    # 模拟 PLC 挂死无响应
                    time.sleep(5)
                    continue

                fn_code = pdu[0]
                resp_pdu = b""

                if fn_code == 0x01:  # Read Coils
                    start_addr, count = struct.unpack(">HH", pdu[1:5])
                    with self.lock:
                        byte_val = 0
                        for i in range(min(count, 8)):
                            idx = start_addr + i
                            if idx < len(self.door_coils) and self.door_coils[idx]:
                                byte_val |= (1 << i)
                    resp_pdu = struct.pack(">BBB", fn_code, 1, byte_val)

                elif fn_code == 0x03:  # Read Holding Registers
                    start_addr, count = struct.unpack(">HH", pdu[1:5])
                    vals = []
                    with self.lock:
                        for i in range(count):
                            idx = start_addr + i
                            if idx < len(self.elevator_regs):
                                vals.append(self.elevator_regs[idx])
                            else:
                                vals.append(0)
                    resp_data = bytearray([fn_code, count * 2])
                    for v in vals:
                        resp_data.extend(struct.pack(">H", v))
                    resp_pdu = bytes(resp_data)

                elif fn_code == 0x05:  # Write Single Coil
                    coil_addr, raw_val = struct.unpack(">HH", pdu[1:5])
                    val = (raw_val == 0xFF00)
                    if coil_addr == 0x0000 and val:  # Open command
                        threading.Thread(target=self._simulate_door_motion, args=("open",), daemon=True).start()
                    elif coil_addr == 0x0001 and val:  # Close command
                        threading.Thread(target=self._simulate_door_motion, args=("close",), daemon=True).start()
                    resp_pdu = pdu  # 回显

                elif fn_code == 0x06:  # Write Single Register
                    reg_addr, reg_val = struct.unpack(">HH", pdu[1:5])
                    with self.lock:
                        if reg_addr < len(self.elevator_regs):
                            self.elevator_regs[reg_addr] = reg_val
                    resp_pdu = pdu  # 回显

                else:
                    # 异常响应 0x80 + fn_code
                    resp_pdu = struct.pack(">BB", fn_code | 0x80, 0x01)

                resp_mbap = struct.pack(">HHHB", tx_id, 0, len(resp_pdu) + 1, unit_id)
                client_sock.sendall(resp_mbap + resp_pdu)
        except Exception:
            pass
        finally:
            client_sock.close()

    def start(self):
        self.running = True
        self.server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_sock.bind((self.host, self.port))
        self.server_sock.listen(5)
        print(f"[MockPLC] Listening on {self.host}:{self.port} (fault_mode={self.fault_mode})", flush=True)

        while self.running:
            try:
                client_sock, _ = self.server_sock.accept()
                t = threading.Thread(target=self._handle_client, args=(client_sock,), daemon=True)
                t.start()
            except Exception:
                break

    def stop(self):
        self.running = False
        if self.server_sock:
            try:
                self.server_sock.close()
            except Exception:
                pass


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Mock PLC Modbus-TCP Facility Server")
    parser.add_argument("--host", default="127.0.0.1", help="Host IP")
    parser.add_argument("--port", type=int, default=15020, help="Port")
    parser.add_argument("--door-delay", type=float, default=1.0, help="Door open delay in seconds")
    parser.add_argument("--fault", default="none", choices=["none", "stuck_closed", "timeout"], help="Fault mode")
    args = parser.parse_args()

    plc = MockPlcFacility(host=args.host, port=args.port, door_open_delay=args.door_delay, fault_mode=args.fault)
    try:
        plc.start()
    except KeyboardInterrupt:
        plc.stop()
