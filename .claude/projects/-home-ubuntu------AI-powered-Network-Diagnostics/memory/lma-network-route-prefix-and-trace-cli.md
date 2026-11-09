---
name: lma-network-route-prefix-and-trace-cli
description: LMA 网络路由全在 /api/v1/network/* 下；trace reporter 的 --incident-id 只支持 sqlite
metadata: 
  node_type: memory
  type: reference
  originSessionId: 5ceebcb5-2ecb-4a68-b482-f59365603b14
  modified: 2026-10-08T13:50:12.147Z
---

`Large-Model-Application` 的 network_assurance 路由 **router 前缀是 `/network`**，
再挂 `/api/v1` —— 即所有端点都是 `/api/v1/network/...`：
`/api/v1/network/assets/{id}/actions`、`/api/v1/network/assets/{id}/wireless-devices/{addr}/risk-prediction`、
`/api/v1/network/assurance/incidents/{id}/council[/proposals]`、
`/api/v1/network/assurance/proposals/{id}/{decision|execute|runbook}`、
`/api/v1/network/edge/{telemetry|wireless-events|action-results}`。
（README / PLATFORM_OPERATION_GUIDE 曾漏写成 `/api/v1/assurance/...`，已修。）

契约要点：
- `POST .../decision` 需 `If-Match: <数字版本>`，body `{"decision":"APPROVED|REJECTED","reason":≥8字符}`；
- `POST .../execute` 需 `Idempotency-Key`；MANUAL 提案一律 409 `manual_execution_not_allowed`；
- `GET .../runbook` 是 **GET**，返回体恒 `execution_status=NOT_EXECUTED`；
- direct queue `POST /network/assets/{id}/actions` 的 body 是
  `{"config_key":..., "config_value":...}`（不是 `{"action":...,"params":{...}}`，后者 422），
  需 `MANAGE_NETWORK_DEVICE`（仅 `TENANT_ADMIN` 有）。

`scripts/verify_end_to_end_trace.py`：
- `--trace-report <json>` → 可 `TRACE OK`（exit 0 仅当 OK）；
- `--incident-id <id> --db-url <url>` → **只支持 sqlite URL**（PG 会 exit 2），
  且结果上限 `PARTIAL`（Prediction 不落库、被拒操作不可事后重建），PARTIAL 也 exit 0。

相关：[[rehearsal-remote-leg-fixture]]、[[cloud-rehearsal-harness]]
