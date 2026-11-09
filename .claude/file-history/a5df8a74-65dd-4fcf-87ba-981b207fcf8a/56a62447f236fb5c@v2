---
name: council-model-schema-timeout-fixes
description: Council 上游两个真实故障的修法——response_schema 必须 minItems=1、upstream 超时默认 90s 太短
metadata: 
  node_type: memory
  type: project
  originSessionId: a5df8a74-65dd-4fcf-87ba-981b207fcf8a
  modified: 2026-10-08T16:29:49.693Z
---

2026-10-09 跑真实 Council 链时踩到并修掉的两个上游问题
（`Large-Model-Application/src/industrial_ops_agent/network_assurance/council_model.py`）：

1. **`COUNCIL_SCHEMA_PARSE_FAILED` / `observations: List should have at least 1 item`**
   - 根因：`_response_schema` 的 `string_array` 只有 `maxItems`，没有 `minItems=1`，
     而 Pydantic 侧 `ExpertOpinion.observations` 是 `min_length=1`。模型事实稀少时
     会返回 `[]`，直连通道没有网关的 `json_schema` 约束，于是 parse 阶段爆掉。
   - 修法：专家角色的 `observations` / `referenced_fact_ids` 用
     `string_array_nonempty`（`minItems: 1`）；协调官 `proposed_preconditions`
     等保持可空的 `string_array`。
   - 直连通道 `UpstreamCouncilClient` 把 schema JSON **拼进 system prompt**，
     所以 schema 改动必须重启 `weaknet-cloud-api.service` 才生效。

2. **`COUNCIL_MODEL_FAILED` / `upstream gateway unavailable or returned no JSON`**
   - 根因：`complete_json` 默认 `_DEFAULT_INFERENCE_TIMEOUT_SECONDS = 90.0`，
     而 Council 是 4 次串行调用、协调官 payload 大（目录 + 3 份专家意见 + 中文
     rationale），单角色 90s ReadTimeout 很常见。
   - 修法：`UpstreamCouncilClient.__init__(timeout_seconds=180.0)` 并透传给
     `complete_json(..., timeout_seconds=...)`（`complete_json` 本来就收这个参数）。
   - 热配置 `model_config.timeout_seconds` 会覆盖默认值。

**Why**：这两个都表现为 fail-closed 409，容易误判成"模型不行"，实际是
schema 约束与超时预算没对齐真实 payload 体量。

**How to apply**：Council 409 时先看 journal 的
`COUNCIL_FAILURE: <CODE>`：`COUNCIL_SCHEMA_PARSE_FAILED` 查 schema/minItems，
`COUNCIL_MODEL_FAILED` 查 timeout 与上游 200 内容。改完**必须重启服务**。
重跑用 `POST .../council?force=true`。

相关：[[rehearsal-remote-leg-fixture]]、[[lma-network-route-prefix-and-trace-cli]]