---
name: lma-attribution-mit
description: LMA 署名与许可已统一为 tanqf/MIT（commit 0c90c45），原声明为 cpp辅导的阿甘/Proprietary，用户确认为其本人持有权利的代码
metadata: 
  node_type: memory
  type: project
  originSessionId: 71256ec0-8b40-4486-a6c7-50fff1561007
  modified: 2026-10-01T03:51:48.427Z
---

用户于 2026-10-01 明确确认 `Large-Model-Application/` 是其本人持有权利的代码，并授权把全部署名统一为 tanqf、许可统一为 MIT。

已完成（commit `0c90c45`，仅含 3 个文件）：
- `Large-Model-Application/README.md` 署名句 → tanqf，许可声明 → MIT
- `Large-Model-Application/pyproject.toml`：`authors = [{name = "tanqf"}]`、`license = { text = "MIT" }`
- 新建根 `LICENSE`（MIT, Copyright (c) 2026 tanqf），修复根 README 中原本失效的 LICENSE 链接

**Why:** LMA 于 2026-09-15 经 `4baa175` 整体导入，原声明为“cpp辅导的阿甘 / Proprietary”，与主仓库 tanqf/MIT 不一致；用户选择统一。
**How to apply:** 后续不要把“cpp辅导的阿甘 / Proprietary”写回任何文件；也不改 git 历史（历史提交作者 dd2006tqf 保留）。若发现新的归属/许可声明残留，以 `LICENSE` + `PROJECT_ATTRIBUTION.md` 为准。相关 [[cloud-integration-division-of-labor]]。
