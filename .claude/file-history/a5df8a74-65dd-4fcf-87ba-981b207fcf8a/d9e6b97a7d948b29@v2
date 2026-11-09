---
name: rehearsal-remote-leg-fixture
description: "2026-10-09 真实 REMOTE 腿已跑通（TRACE OK）；旧的\"远端腿只能 fixture\"结论作废"
metadata: 
  node_type: memory
  type: project
  originSessionId: a5df8a74-65dd-4fcf-87ba-981b207fcf8a
  modified: 2026-10-08T16:29:49.656Z
---

**2026-10-09 更新：远端腿已经真实跑通，本条记忆的历史结论作废。**

真实闭环（incident `sitinc_radxa-cubie-a7a_1791472968`，全部非 fixture）：
- council `ncouncil-5f5180a7598a48f0`（attempt 3）真实模型产出
  `CONFIG_CHANGE rssi.interval=1000` 提案 `ncprop-5f5180a7598a48f0-a3-2`
  （`proposal_index=2`，digest `sha256:c810edaab723…`，在 `proposals_json` 内）
- decision `ncdec-29446c303418f82db83b9ed0`（ALLOWED/LOW/REMOTE_PENDING_ACTION）
- approval `ncapp-4df6b44e3ed926f45d0a9a68`（APPROVED，`second-engineer-approver`）
- pending `nact-3642ed5382964233ba301d58664a1e18`（APPLIED，gen 3）
- outcome `nout-e850c9d725fd4eeda6be9b2c8080769d`（APPLIED）
- 报告器 `--trace-report` 判定 **TRACE OK**（9 edge + 3 执行 + 5 拒绝断言全过）
- 板端 `weaknet-cli get rssi` 实测 `interval_ms=1000`

**Why**：旧结论"真实模型只产 ACTION_ID、远端腿结构性不可得"是**该次会商**
（超时类 incident、schema 未加 minItems、max_tokens 4096）的观察，不是结构约束。
换 incident 后模型自行产出了 `CONFIG_CHANGE`。

**How to apply**：
- 判断某条 REMOTE 链是否 fixture：核 `proposal_id` 形如
  `ncprop-<后缀16>-a<attempt>-<index>`、`proposal_index` 在 `proposals_json`
  范围内、digest 非占位串——本条新链三条全满足。
- 旧 fixture（`ncouncil-10d3ed70cf6749b1-remote-config`）仍在库中，
  只是不再是唯一 REMOTE 证据。
- 入口诚实边界：本 incident 的**事件载荷是签名注入**（物理断连会被
  `QualifyingAnomalyPolicy` 排除 + `min_affected_devices>=2` 双门槛），
  走的是真实 Ed25519 签名链路与真实 ingestion，但**不含现场物理断连**。

相关：[[cloud-rehearsal-harness]]、[[edge-downlink-rehearsal]]、[[lma-network-route-prefix-and-trace-cli]]