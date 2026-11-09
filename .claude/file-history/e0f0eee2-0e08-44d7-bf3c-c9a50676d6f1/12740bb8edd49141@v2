---
name: phase2-bt-l2-anomaly-detector
description: 蓝牙 Phase 2 L2 异常检测器已板端实测；三种规则、上报抑制、死句柄清理的实现要点与环境限制
metadata: 
  node_type: memory
  type: project
  originSessionId: e0f0eee2-0e08-44d7-bf3c-c9a50676d6f1
  modified: 2026-10-13T03:23:36.026Z
---

蓝牙工业数传 Phase 2（规则驱动 L2 异常检测）于 2026-10-13 完成开发并在 Radxa 板端实测（commits 994d334、0cdf1b9）。

**实现要点：**
- 三种规则：`BT_DATAPATH_STALL_SUSPECTED`（仅限 device 归因连接）、`BT_DATAPATH_THROUGHPUT_DEGRADED`、`BT_DATAPATH_ATTRIBUTION_LOST`；检测器在 `BtHciTrafficPlugin` 每 2 秒采样循环内运行，配置键 `bluetooth.stall_timeout_ms / stall_consecutive_windows / min_expected_bps / degraded_consecutive_windows`。
- 上报抑制：每异常季只发 1 条 WARNING + 1 条 CRITICAL 升级（曾实测每 2 秒刷一条日志+事件）。
- 三个板端踩过的坑已修：handle=0 pending page 被当活跃连接（STALL 无限误报）、死句柄 BPF Map 残留不清理（64 槽位耗尽+幻影 controller 归因观测）、内核连接表须每 tick 同步（1.3µs 实测）否则 prune 会误删新连接。
- 异常桥接为 `LINK_DEGRADED` 事件（`EvidenceSource::Derived`、`source_detail=异常名`、完整证据在 details_json），`weaknet-cli events --type LINK_DEGRADED` 可查。

**环境限制：** 板端射频环境的蓝牙设备大多"可发现不可连接"（page timeout / RSSI -88），`THROUGHPUT_DEGRADED` 的真机实时触发需要用户把手机/耳机置于近距离可连接状态；`l2ping` 失败的 page 尝试会产生 handle=0 内核记录属正常现象。

相关：[[phase2a-dns-implementation]]、[[single-source-capture-principle]]、[[board-network-access]]
