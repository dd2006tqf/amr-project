---
name: cloud-rehearsal-harness
description: "板↔云端排练的可复用手法（免 compose.lite 的最小服务 + 板端反向端口转发），以及 0089 迁移缺列这类\"SQLite 测试测不出\"的盲区"
metadata: 
  node_type: memory
  type: project
  originSessionId: 71256ec0-8b40-4486-a6c7-50fff1561007
  modified: 2026-10-01T10:25:54.311Z
---

2026-10-01 首次跑通 WeakNet 板端 → Large-Model-Application 的**无线事实上行 → 云端诊断**端到端排练。可复用手法：

## 网络：板端反向端口转发（不用隧道端口共享即可）
云端没有板子的入口，且 sshd `GatewayPorts no`。但板 → 云可达，于是：
```bash
ssh board 'nohup ssh -N -L 8000:127.0.0.1:8000 -i ~/.ssh/id_cloud_tunnel ubuntu@81.71.76.133 &'
```
板端 `127.0.0.1:8000` 即指向云端 8000。板端 `edge.url` 保持 `http://127.0.0.1:8000/api/v1/network/edge/telemetry`——**不要**改成 `/wireless-events`：
`EdgeWirelessUplinkExporter::uplinkUrl()` 是靠替换 `/telemetry` 后缀派生端点的，
把 telemetry 端点本身改成 wireless-events 会让派生失败（返回空 URL → 500）。

## 云端：最小排练服务，不启 compose.lite
本机 3GB 内存，compose.lite 要 ~20 个容器；且全量 alembic 链（0000→0090）在 pgvector 镜像上能跑。
用 `Large-Model-Application/scripts/rehearsal_server.py`：只挂 `network_assurance` 路由 + 一个真实
`Authorizer` + `EdgeTelemetryVerifier`，attest 到临时 pgvector 容器（55432）。
它给 `industrial_ops_agent.api.dependencies` 打了一个 shim（真实模块会把整个平台拖进来：
Presidio/spaCy、MLflow、Kafka…）。**shim 的坑**：FastAPI 解析依赖签名时，未注解的 `request`
参数会被当成 query 参数（422 `missing query.request`）；而 `from __future__ import annotations`
下类型注解在**模块 globals**里解析，函数内 import 无效——必须在文件顶层 `from fastapi import Request`。

## 盲区教训：hermetic 单测通过 ≠ 迁移可跑
`SiteIncidentDiagnosisRecord` 继承 `TimestampMixin`（created_at + **updated_at**），但迁移
`0089_wireless_incident_diagnosis` 只建了 created_at。SQLite 测试用 `Base.metadata.create_all`
从模型建表，永远走不到迁移，于是两个月的绿灯掩盖了"PG 上每条诊断查询都 UndefinedColumn 500"。
**新增表时，迁移里的列必须与模型逐列对齐**；只跑 SQLite 测试不够。

## 板端密钥获取
板端私钥不导出（正确）。需要以板端身份签名时，从板端取**公钥**即可：
`ssh board 'sudo openssl pkey -in /home/radxa/weaknet/config/edge_priv.pem -pubout'`。

相关：[[edge-downlink-rehearsal]]、[[cloud-integration-division-of-labor]]
