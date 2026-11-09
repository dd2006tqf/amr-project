---
name: workflow-harness-snapshot-bug
description: change_abort.sh 曾 delete active_change 字段导致快照永久失效；已修为显式置 null
metadata: 
  node_type: memory
  type: project
  originSessionId: c18c9c46-652e-4c79-8de2-41f56df36b24
  modified: 2026-09-24T16:27:20.890Z
---

`scripts/change_abort.sh` 第 50 行曾用 `delete d.active_change` 清理 root snapshot，但 `active_change` 是**必需字段**（schema 允许值为 null，但键必须存在）。

**Why:** `harness_doctor.js:46` 与 `harness_lock.sh:110` 的 `harness_active_optional()` 都校验 `active_change === null || typeof active_change === 'string'`，键缺失直接判 `active selector is invalid`（exit 6）。后果是 `change_select.sh --clear` 等所有托管包装器全线失败——因为 `--clear` 分支先调 `harness_active_optional` 才写回 null，形成死锁：越坏越修不了。git 历史里 `4fc10b9`/`1a42f94`/`8e7cde9` 三次 "fix(harness): 恢复快照 active_change 字段" 就是同一个 bug 反复发作。

**How to apply:** 2026-09-24 已修：改为显式 `d.active_change=null`，并加 schema 前置校验 + tmp+rename 原子写。修复后 `harness_doctor.sh` 32/32 全绿。如果以后再看到 `active selector is invalid`，先检查 `ai_snapshot.json` 是否有 `active_change` 键（`python3 -c "import json;d=json.load(open('ai_snapshot.json'));print('active_change' in d)"`），而不是去查 change 目录。

另一个同源陷阱：`scripts/change_abort.sh` 与 `scripts/auto_archive_push.sh` **不在** manifest 的 managed_paths 里（manifest 只登记 83 个固定模板路径，无 glob），所以 script 本身的 bug 不会触发任何校验。

相关：[[project-profile-cmake-migration]]
