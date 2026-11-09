---
name: edge-downlink-rehearsal
description: Cloud→Edge 下行通道（pending_actions）真机联调结果与反向隧道联调手法
metadata: 
  node_type: memory
  type: project
  originSessionId: 3a327297-8e4a-4107-9089-d54a01d9f684
  modified: 2026-09-16T17:29:55.887Z
---

2026-09-17 Cloud→Edge 下行通道真机联调**已完整走通**（云端下发 → 边缘白名单校验 → 热应用 → 结果签名回传 → DB 落 APPLIED，全程约 400ms）。

- **补上的缺口**：边缘 C++ 原先只应用下发动作用不回执 —— `pending_action_results_` 队列在 `edge_telemetry_exporter.hpp` 里声明了但全仓库无人写入、无人发送，云端 action 永远停在 `DELIVERED`。已补 `EdgeActionResultRecord`、`queueActionResult`、`transmitActionResults`、`actionResultsUrl`（从 telemetry URL 换尾段派生 `/action-results`）、`postSigned`。
- **联调网络手法（关键）**：云主机 `192.168.3.100`（ens160）与开发板 `192.168.137.210`（电脑热点）**跨广播域**，云→板通（走 3.x 网关绕回热点 NAT），但**板→云不通**（热点不转发到宿主 LAN，实测 tcpdump 在 ens160 上一个包都收不到）。mDNS/`.local` 解决不了（链路本地组播不跨段）。可行手法是**反向隧道**：`ssh -N -R 18000:192.168.3.100:8000 radxa@radxa-cubie-a7a.local`，板端 `edge.url` 指 `http://127.0.0.1:18000/...`。
- **运维注意**：`edge.url` 刻意不在运行时调参白名单内（`weaknet_config.cpp:644` 注释说明：防止远程把遥测重定向到别处），改它必须编辑 `/etc/weaknet/config.yaml` 并重启服务。
- **复现一次下发**：直接向 `network_pending_actions` 插一条 `status='QUEUED'` 的行即可（需带 `tenant_id`/`asset_id`/`issued_by`/`issued_at`/`version`），板端会在下一个上报周期取走。
- **上行仍有一处既存缺陷**见 [[edge-uplink-epoch-collision]]。

相关：[[board-network-access]]、[[edge-uplink-epoch-collision]]
