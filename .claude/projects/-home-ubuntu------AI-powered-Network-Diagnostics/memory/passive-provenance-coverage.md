---
name: passive-provenance-coverage
description: Active Probe 自流量排除只覆盖 DNS/TCP 两个探针，HTTP 与 retransmit 未覆盖
metadata: 
  node_type: memory
  type: project
  originSessionId: 9f6f73c9-45d6-45d3-ada8-bae10cd4eed4
  modified: 2026-09-13T13:56:12.317Z
---

Active Probe 自身流量排除的覆盖现状（2026-09-13 复核）：

- 已覆盖：`dns_monitor.bpf.c`（`dns_self_pid` map + `dns_self_socks` LRU_HASH，egress 写 cookie、ingress 按 cookie 排除）、`tcp_connect.bpf.c`（`tcp_self_pid` + `bpf_get_current_pid_tgid() >> 32` 过滤）。
- **未覆盖**：`http_latency.bpf.c`、`tcp_conn_stats.bpf.c`、`tcp_retransmit.bpf.c`、`flow_rate.bpf.c`、`skb_drop.bpf.c`、`wifi_packet_loss.bpf.c` —— 全都没有 self-traffic 排除。

这条对 W1 的验收有直接影响：一旦 Active Probe 开始发 HTTPS 明文之上的 TLS 流量或明文 HTTP 流量，`http_latency` 与 `tcp_retransmit` 窗口就会被探测流量污染，而 "仅开探测时 passive 计数零增长" 的硬验收会失败。所以 provenance 统一化必须与 Active HTTPS 同步落地，不能留到最后。

相关：[[single-source-capture-principle]]
