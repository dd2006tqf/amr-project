---
name: working-tree-separation-discipline
description: 从混杂工作树里拆分提交前，必须先把整份 git diff 落盘备份，否则不可恢复
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 6a40e643-c3c3-49a0-ae96-1b0f7e7e3a27
  modified: 2026-10-30T08:26:25.132Z
---

2026-10-30 的事故：为把相位 11 与一组硬件改动分开提交，我按"提交一部分、对其余 `git checkout -- <file>`"的办法清理工作树，结果把未提交的硬件改动（chassis 协议/电池/链路存活、path_tracker odom 原点）**永久丢失**——没有 stash、没有备份 diff，git 里也没有对应 blob。最后靠 `.claude/projects/*/*.jsonl` 会话记录里原样捕获的 `git diff` 输出逐条 `patch` 才恢复（commit `2f71a5d`）。

**Why**：未提交的工作树改动没有第二份副本。`git checkout --` / `git restore` 是不可逆的，而"我知道那部分不重要"这个判断在混合改动里经常是错的——这次被删的恰恰是完整功能集。

**How to apply**：
1. 拆分提交前先 `git diff > /tmp/split_full.diff`（以及 `git status --short`），再动任何 `checkout`/`restore`。
2. 更安全的是把不属于本次提交的路径挪到临时目录（`cp` 出去）而不是 `checkout` 回去。
3. 万一丢了：先查 `git stash list` / `git fsck --lost-found` / 各分支，最后才查 `~/.claude/projects/<proj>/*.jsonl` 里的 tool_result（`git diff` 的输出原样存在那里，含行号），用 `patch` 回打——恢复后必须重新编译 + 跑相关 e2e 证明恢复正确，而不是只看 diff 像不像。

相关：[[phase11-ops-observability]]
