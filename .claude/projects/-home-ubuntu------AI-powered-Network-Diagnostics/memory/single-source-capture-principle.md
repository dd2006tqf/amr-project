---
name: single-source-capture-principle
description: eBPF 捕获必须单一来源——endpoint 与 payload 同 hook 取齐，禁用跨探针 join；选点需按内核语义逐个验证
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 9d6ab6dd-2294-42dd-a2f0-cacbcc00538d
  modified: 2026-09-12T05:00:19.833Z
---

2026-09-12 用数十轮迭代换来的架构结论：**一个 observation 的所有字段必须来自同一个 hook / 同一语义源**，禁止跨独立探针按时序 join。

**Why:** 跨探针 join 需要猜关联键（pid_tgid 等），在并发/交错场景下不可靠，还会留下陈旧条目污染后续事件。当时 DNS 出现 `tracker_q=45 vs tracker_r=324`，反复修 ABI、内存域、字节序都无法收敛；改为单一来源后一次到位（`ok=18 unmatched=0`）。

**How to apply:**
- 选挂点时先问"这一次事件里能不能同时拿到全部字段"，而不是"能不能凑出来"。
- 已实测可行的单一来源点：
  - DNS query → `kprobe/ip_finish_output2`（skb 带完整 UDP/IP 头）
  - DNS response → `kprobe/udp_queue_rcv_skb`（skb 带完整载荷）
  - TCP 建连 → `tracepoint/sock/inet_sock_set_state`（四元组 + 状态迁移同源）
- **不行的挂点（已证伪，别再试）**：`udp_sendmsg` 入口（autobind 未完成，`skc_num`/`skc_rcv_saddr` 为 0）；`udp_send_skb` 入口（UDP 头尚未写入）。
- 两处内存域细节，写错必然静默失败：
  - `iovec` descriptor 在内核内存（`iovstack`/kmalloc），只有 `iov_base` 指向用户数据；
  - `sk_buff` 用 `head + transport_header/network_header` 定位，不要用会随 `skb_pull` 移动的 `data`。
- TCP 建连关联键**不能含临时端口**：SYN_SENT 时 `sport=0`，端口在 ESTABLISHED 才分配（ftrace 实测）。
- 任何失败都必须落在一个明确的 counter 上再动手；不能给出确定边界就换方案，不要继续堆补丁。

相关：[[phase2a-dns-implementation]]
