---
name: phase2a-dns-implementation
description: Phase 2A DNS Service Assurance 已实施——raw_syscall 探针需显式 attach（曾致 NO_DNS_EVENTS）；resolver 用 sendmsg/recvmsg；PARTIAL 噪声是已知边界
metadata: 
  node_type: memory
  type: project
  originSessionId: 9d6ab6dd-2294-42dd-a2f0-cacbcc00538d
  modified: 2026-09-11T15:28:51.883Z
---

2026-09-11 完成 Phase 2A（DNS Service Assurance）实施与真机验证（工作流外，用户确认不走 change）。验证记录见 `docs/Phase2A-DNS-Service-Assurance-验证记录.md`。

**Why:** 排查 NO_DNS_EVENTS 走了三轮弯路，这些结论决定后续 DNS/类似 eBPF 事件链路的排障方向。

**How to apply:**
- **libbpf 对象中非 SEC 入口的 tracepoint 程序不会自动 attach**：必须显式 `bpf_object__for_each_program` + `bpf_program__attach`。怀疑"probe 没触发"时先 `sudo journalctl -u weaknet-server | grep "capture programs attached"`。
- **开板 resolver 实际用 sendmsg+recvmsg**（sendto/sendmmsg 为 0），已通过 raw_syscall tracepoint + arm64 syscall id 门控覆盖（sendto=206/recvfrom=207/sendmsg=211/recvmsg=212/sendmmsg=213/recvmmsg=243）。
- **诊断计数器链路已建成**：`dns-capture diag` 日志输出 BPF per-CPU counters（msghdr/iovec/payload/emit 分级失败）+ drain counters（poll/sample/lost/tracker_accepted），可精确定位事件丢失边界，不要再猜 ABI。
- **perf buffer 8 页在高频 syscall 下会丢事件**（emit_fail≈2872），需要扩页。
- **PARTIAL 事件噪声是已知边界**：capture 无法关联 fd→endpoint，query/response 都以 TxID-only 匹配，failure_ratio 被稀释，DNS 判定持续 BAD。改进方向在验证记录"已知边界"节。
- SQLite WAL sidecar（-shm/-wal）必须与主库同属主同权限，否则报 attempt to write a readonly database。

相关：[[arm64-container-binfmt-reset]] [[board-network-access]]
