---
name: bluetooth-stack-architecture
description: 弱网项目中蓝牙协议栈的真实架构——BlueZ 用户态控制面 + Linux 内核原生 net/bluetooth 传输层 + 随开发板而定的硬件 Controller（本仓库板实测 AICSemi AIC8800D80，非博通）
metadata: 
  node_type: memory
  type: reference
  originSessionId: 530b10c6-9158-49e3-b9d9-9fffc9b6b305
  modified: 2026-10-11T02:12:46.522Z
---

在 WeakNet 弱网诊断项目中，蓝牙模块是**开源的 Linux 双层协议栈架构**，硬件 Controller 的厂商取决于所用开发板：

1. **用户态控制面（Upper Host Stack）— BlueZ 守护进程 (`bluetoothd`)**：
   - 职责：负责 GAP、GATT（BLE）、SDP、Profile 连接管理（A2DP 等）。
   - 交互方式：WeakNet 服务端（`bt_monitor.cpp`）作为 D-Bus 客户端，通过系统总线 `org.bluez` 接口监听属性变更与事件，完成双模（经典蓝牙 BR/EDR 与 BLE）设备发现、配对态维护以及平滑 RSSI 的周期性采集。

2. **内核态传输面（Lower Host Stack）— Linux 原生内核蓝牙子系统 (`net/bluetooth/`)**：
   - 职责：负责 L2CAP 逻辑链路分片/重组/流控、HCI 核心数据分发与超时状态机调度。
   - 交互方式：项目中的 eBPF 探针（`bt_events.bpf.c`、`a2dp_media.bpf.c`）直接挂载在内核层（kprobe 挂载 `mgmt_device_disconnected`、`hci_disconnect`、`l2cap_sock_sendmsg` 等），用于无侵入抽取微秒级断连 Reason Code 与传输时延。

3. **底层硬件层（Controller）— 随开发板而定的板载蓝牙模组**：
   - Controller 跑在硬件或原厂固件里，通过 USB、UART 或 SDIO 向上暴露标准 HCI 接口。**厂商是硬件选型，不是技术栈的一部分，也不应被固定成某个品牌**（博通走 `btbcm`、瑞昱走 `btrtl`、英特尔走 `ibtusb`，均不同）。
   - **本仓库开发板实测（Radxa Cubie A7A）**：Wi-Fi/BT 二合一模组 **AICSemi AIC8800D80**（`lsusb` 显示 `a69c:8d81`），BT 驱动 `aic_btusb`，SoC 为全志 A733（`sun60iw2`）。

**面试与对外表达核心纪律**：
- 不要说“底层物理链路跑了厂商私有协议栈”——Host 侧永远是标准 Linux 内核 + BlueZ；
- 不要把芯片品牌当作项目技术点，被问到时如实答“取决于实际平台”再补本机实测型号；
- eBPF 挂钩的是通用的 Linux 内核符号，与射频芯片厂商完全解耦，换芯片不用改探针。

相关：[[board-bluetooth-no-tracepoint]]、[[wireless-event-model-phase1]]
