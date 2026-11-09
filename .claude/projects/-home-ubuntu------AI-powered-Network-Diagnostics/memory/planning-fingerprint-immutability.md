---
name: planning-fingerprint-immutability
description: 记录 evidence 前不能修改 design.md，否则旧证据的 planning fingerprint 失效导致 --complete 失败
metadata: 
  node_type: memory
  originSessionId: cdab7a5b-9099-4349-a685-bb0d04203f70
---

一旦开始用 task_verify.sh 记录 evidence，就绝对不能修改 design.md（除非重新冻结 planning baseline 并重跑所有 evidence）。

**Why:** 任何 design.md 变化都会改变 planning_fingerprint 和 tdd_policy_sha256，导致之前所有 evidence 中的 planning_fingerprint_before/after 变成旧值，verifyTddEvidenceAt 以 "planning changed during command" 拒绝所有任务完成。

**How to apply:**
- 实施前：先确认 design.md 的所有修改（classification、exceptions、thresholds）已完成并冻结
- 实施后：只改代码文件和 tasks.md（勾选 checkbox），不碰 design.md
- 如果必须改 design.md：需要运行 sync_hashes.sh 重新同步，然后对所有任务重新跑 task_verify.sh 记录证据（旧命令无法修复）
- 修改 design.md 前先检查当前 change 是否已有 evidence 记录，如果有则警告用户后果
