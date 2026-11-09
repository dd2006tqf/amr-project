---
name: edge-uplink-epoch-collision
description: 板端重启后上行遥测被服务端静默去重丢弃的根因与修法（network_epoch 持久化）
metadata: 
  node_type: memory
  type: project
  originSessionId: 3a327297-8e4a-4107-9089-d54a01d9f684
  modified: 2026-09-16T17:29:38.260Z
---

2026-09-16/17 下行联调时发现的既有缺陷：**板端每次重启后，上行遥测全部被服务端当重复丢弃，返回 HTTP 200 不报错。**

- **根因**：幂等键是 `(tenant, device, network_epoch, sequence_id)`。`sequence_id` 来自 `ServerContext::assessment_sequence`（`server.hpp:109`，进程内 atomic，初值 0，每次重启从 1 重来）；`network_epoch` 来自 `dns_binding_epoch`（`server.hpp:85`，初值 1，只在路由/解析器变化时推进）。两者都是进程内计数器，重启后 `(epoch=1, seq=1..N)` 与上一轮的历史键完全重合 → `duplicates=N` → 数据丢弃。
- **表现极具迷惑性**：API 日志全是 200，板端一条"上报失败"都没有，但 `network_snapshots` 不再增长。只能靠抓包比对实际发出的 `sequence_id` 与库中已有键来定位。
- **诊断手法**（可复用）：`sudo tcpdump -i any -A -s0 -n 'tcp port 8000' | grep -aE 'sequence_id|network_epoch'`，再与库中 `SELECT ... WHERE network_epoch=? AND sequence_id=?` 对照。
- **修法**（已实现）：新增 `server/include/network_epoch_store.hpp` + `server/src/network_epoch_store.cpp`，启动时 `open()` 读取 `data_dir/network-epoch`、+1、原子写回（tmp + rename），并同时初始化 `dns_binding_epoch` 与 `dns_tracker->advanceBindingEpoch()`。首次运行返回 1（与既有历史不冲突）；状态文件损坏返回 `kRecoveryEpoch=1000000` 而非回落 1（回落会再次撞历史键）。
- **注意**：曾考虑"从本地 history.db 恢复 sequence_id"，**该方案不成立** —— `network_history` 表根本没有 `sequence_id`/`network_epoch` 列，且该表只在 `snap_valid` 时写入，恰好在重启窗口不写。

相关：[[board-network-access]]、[[edge-downlink-rehearsal]]
