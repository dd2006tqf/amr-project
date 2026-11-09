---
name: claude-code-single-source-config
description: Claude Code 配置唯一事实源是 ~/.claude/settings.json 的 env 段，shell rc 里不再放任何 ANTHROPIC_/CLAUDE_CODE_ 变量
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 0156e006-6ff3-4d05-9497-dea89245026a
  modified: 2026-10-30T15:12:39.262Z
---

用户要求 Claude Code 的所有 API/模型/运行参数只在一个地方维护：**`~/.claude/settings.json` 的 `env` 段**。`~/.bashrc` 里不再有任何 `ANTHROPIC_*` / `CLAUDE_CODE_*` 赋值（2026-10-30 已全部迁走）。

**Why:** 之前两处都配（`~/.bashrc` + `settings.json`），改一处另一处还在生效，导致 `/model` 选 default 却被 shell 里的 `ANTHROPIC_MODEL` 悄悄压住。实测确认：`settings.json` 的 `env` **优先级高于进程环境变量**，所以它足以单独充当事实源；这还顺带挡住了 IDE/旧终端里手动 export 的脏变量。

**How to apply:**
- 改 API 地址、密钥、模型别名映射 → 只改 `~/.claude/settings.json`。
- `"ANTHROPIC_MODEL": ""` 是**刻意**留着的空串，用来中和父进程残留的同名变量（实测有效）。不要"清理"成删除或写成具体模型名——写成模型名就会重新 pin 主模型，`/model` 的 default 又失效。
- 不要设 `"model"` 键，让 `/model` 的 default 生效（实测不设任何模型变量时 default 解析为网关当前模型）。
- `ANTHROPIC_DEFAULT_OPUS|SONNET|HAIKU_MODEL` 只做别名解析，指到网关真实模型名即可，不 pin 主模型。
- `CLAUDE_CODE_SUBAGENT_MODEL` 是独立开关，实测生效（子代理走 `deepseek-v4-pro-0813`）。
- 改完 `settings.json` 后需重启会话生效；改 `.bashrc` 则需新开终端。

相关：[[claude-code-default-model]]
