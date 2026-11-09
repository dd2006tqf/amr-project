---
name: board-bluetooth-no-tracepoint
description: 板端蓝牙无 tracepoint、BT 是模块、vmlinux.h 无蓝牙结构；只能用 kprobe + 手算偏移
metadata: 
  node_type: memory
  type: project
  originSessionId: d3a9e2e5-7ae5-4b9b-85f4-eefa0f94d189
  modified: 2026-09-29T13:33:20.453Z
---

目标板（Radxa Cubie A7A，内核 `5.15.147-99btf1-a733`）的蓝牙 eBPF 可行性，2026-09-29 全部实测：

- **没有 bluetooth tracepoint**：`/sys/kernel/debug/tracing/events/bluetooth/` 不存在，tracefs events 下也没有。任何"tracepoint → fentry → kprobe"的降级链在这里第一步就是空的，**只能 kprobe**。
- `CONFIG_BT=m`：蓝牙是模块，`hci_conn`/`l2cap_chan`/`hci_dev` **不在 vmlinux BTF 里**（`board-assets/vmlinux.h` 零命中），所以**不能 CO-RE 读蓝牙结构**，必须自定义最小结构 + `bpf_probe_read_kernel` + 容错。
- `/sys/kernel/btf/bluetooth` 本身**有效**（77101 B，magic `0xeb9f`）。板端 bpftool 报错是老版本工具问题，不是 BTF 坏。
- kprobe 符号齐全：`mgmt_device_disconnected`、`hci_disconnect`、`hci_conn_del`、`hci_conn_timeout`、`hci_event_packet`、`l2cap_sock_sendmsg` 等。

**Why:** 实测与仓库文档里"tracepoint > fentry > kprobe"的假设直接冲突（`docs/蓝牙监控优化实现方案.md` 4.5.4 节）。照文档写会在板子上全挂不上。

**How to apply:**
- 首选挂点 `mgmt_device_disconnected` —— 内核 mgmt 层向 bluetoothd 通报断连的统一出口，参数直接带 reason，比解析 HCI skb 稳。签名（BTF 实锤）：`(hdev, bdaddr, link_type, addr_type, reason, mgmt_connected)`。
- 结构偏移（BTF 实锤，别猜）：`hci_conn.dst`=20、`dst_type`=26、`type`=55、`hdev`=1400；`hci_dev.id`=64、`bdaddr`=68；**`l2cap_chan.dst`=33**。
- **`bdaddr_t` 是反序存储**：`b[0]` 是最低字节，显示要 b[5]..b[0]（对照 BlueZ `ba2str()`）。踩过这个坑——连接 `AA:BB:CC:DD:EE:01` 被记成 `01:EE:DD:CC:BB:AA`。
- 顺带发现：现有 `a2dp_media.bpf.c` 把 `l2cap_chan.dst` 硬编码成 +24，实际是 33，其 bdaddr 提取大概率是错的（未修，已登记在文档 11.4 节）。

相关：[[single-source-capture-principle]]、[[board-network-access]]
