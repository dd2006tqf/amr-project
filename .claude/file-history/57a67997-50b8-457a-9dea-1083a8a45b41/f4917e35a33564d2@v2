---
name: claude-code-proxy-selfkill
description: 本机 Claude Code 走 127.0.0.1:8787 本地代理，pkill -f 会自匹配并掐断自己与模型的通信
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 57a67997-50b8-457a-9dea-1083a8a45b41
  modified: 2026-11-02T04:57:40.327Z
---

本机 Claude Code 的 `ANTHROPIC_BASE_URL` 是 `http://127.0.0.1:8787`，由 systemd user 单元
`laneai-usage-proxy.service` 提供服务（脚本 `~/.claude/laneai-usage-proxy.py`）。代理在网络
路径上位于本会话自己的 API 调用之前，杀掉它 = 本会话立刻失去与模型的通信能力。

2026-11-02 我执行 `pkill -f 'laneai-usage-proxy.py'` 时踩了两个坑：一是这条命令的模式串
匹配到了正在执行 pkill 的 `bash -c` 自身命令行，进程被杀，工具返回 exit 144；二是它同时
杀掉了当时正在承载本会话请求的代理进程，导致后续数轮用户消息全部 Connection refused，
用户无法与我对上话。

**Why:** 这是"修改承载自己的通道"类的自伤操作。工具返回值只有一个 exit code，杀进程的
一瞬间不会报错，真正的代价出现在下一轮对话——而那一轮已经发不出去了。

**How to apply:**
- 绝不在会话内用 `pkill -f` 处理该代理；需要换代码时用
  `systemctl --user restart laneai-usage-proxy.service`（原子重启，秒级恢复，
  Claude Code 侧有 `CLAUDE_CODE_MAX_RETRIES=10` 兜底重试）。
- 任何"重启/替换当前网络链路上组件"的操作，先自问该组件是否在承载本会话流量；是则
  选择原子切换而非 kill-then-start，并把验证命令写在同一条命令的后半段。
- 排查入口：`systemctl --user status laneai-usage-proxy`、`journalctl --user -u
  laneai-usage-proxy`、`~/.claude/laneai-proxy.log`。`loginctl show-user ubuntu -p
  Linger` 必须为 `yes`，否则退出登录后用户服务被清理，代理消失。
- 背景：该代理存在的目的是给不上报 usage 的中转站补 `input_tokens`，让
  [[planning-fingerprint-immutability]] 无关的自动压缩能正常触发。
