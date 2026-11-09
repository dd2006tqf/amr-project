---
name: concurrent-claude-sessions-same-repo
description: 本机常驻多个 claude 会话共用同一仓库/worktree，会出现非本会话的未提交改动——提交前必须逐个核对文件
metadata: 
  node_type: memory
  type: project
  originSessionId: 5ceebcb5-2ecb-4a68-b482-f59365603b14
  modified: 2026-10-08T16:07:19.827Z
---

2026-10-08 深夜实测：`ps -ef | grep claude` 显示**同一台机器上同时有 6 个 claude
会话在跑**（pts/1–6），共用同一个仓库根
`/home/ubuntu/个人学习/AI-powered-Network-Diagnostics`。

后果：工作区会出现**不是本会话产生的**未提交改动。本次实例：
`Large-Model-Application/src/industrial_ops_agent/network_assurance/council_model.py`
在 00:01:50 被改动（给专家角色的 `observations`/`referenced_fact_ids` 补
`minItems: 1`，与 `council_contracts` 的 `min_length=1` 对齐），而该会话只读过
这个文件、从未编辑。

**做法**：
- 提交前**逐个核对** `git status` 里的文件，确认每一项都是自己改的；用
  `git add <明确路径>` 而不是 `git add -A`——后者会把别人的半成品一起提交。
- 遇到不明改动先查 mtime（`stat -c '%y %n'`）与时间线是否有交集，再决定，
  不要盲目提交或删除。
- 这与 CLAUDE.md 铁律「Keep one writer per worktree」一致：并发会话只应做
  只读调查或独立评审；真要写，需用户批准隔离的 worktree。

相关：[[cloud-integration-division-of-labor]]、[[lma-nested-workflow-dormant]]
