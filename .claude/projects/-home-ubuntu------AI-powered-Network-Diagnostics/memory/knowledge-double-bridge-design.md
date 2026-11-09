---
name: knowledge-double-bridge-design
description: S8 知识双向的落地边界——创建/审核/发布三段分离、知识只做背景不做证据、以及平台既有的几个硬门禁
metadata: 
  node_type: memory
  type: project
  originSessionId: 71256ec0-8b40-4486-a6c7-50fff1561007
  modified: 2026-10-01T17:21:07.220Z
---

2026-10-01/02 落地 P6（S8 知识双向）。三条边界必须保持，改代码前先读：

**1. 三段分离，自动化只能碰第一段**
- 创建：`Action.CREATE_KNOWLEDGE_DRAFT`（新窄动作，授予 FIELD_ENGINEER/DOMAIN_EXPERT/TENANT_ADMIN，
  **不进 OPA 强制集**，与 `CREATE_INCIDENT_DRAFT` 先例一致）。`create_document` /
  `create_extracted_document` 用它；`create_version`（改既有文档）与 review/build/promote
  仍要 `PUBLISH_KNOWLEDGE`。
- 审核：平台强制 `created_by != reviewed_by`（`knowledge_review_separation_required`）。
- 发布：`build_release`（要 Temporal 跑评估）→ CANDIDATE → promote → PUBLISHED。
  **全程不自动化**，人来做。

**2. 知识只做"背景"，绝不进证据集**
检索结果注入 `payload["knowledge_context"]`，`evidence_catalog` / `valid_evidence_ids`
永远只含本案事件——W6 护栏因此对"拿知识 id 当证据"依旧 BLOCKED。
检索 query 用 `pattern + hypothesis` 枚举值原文（tokenize 与文档同源）。

**3. 归档只收确定性内容**
`build_case_content` 在 `presentation.llm_used=True` 时**排除模型报告**，只留事实+规则结论。
把生成文本洗成可检索"历史"，下一个诊断会引到它。

**平台侧硬门禁（踩过的）**
- `source_uri` 必须 https/s3/minio → 用 `network_public_base_url` 配置指向真实诊断资源；
  未配置则**跳过建草稿**，不伪造 URL。
- 检索要 **PUBLISHED + is_active** 的 release；`RetrievalQuery.device_model` 必填（用 "weaknet-gateway"）。
- ACL 四类 AND；空列表=不限制；`classification="restricted"` 必须给角色。
- `lock_subject` 只认 `status=="active"`（小写），但 `tenant_administration` 写的是 `"ACTIVE"`——
  自动化主体由 `network_assurance/automation_subject.py` 按小写 provision。

相关：[[cloud-integration-division-of-labor]]、[[cloud-rehearsal-harness]]
