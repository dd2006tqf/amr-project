---
name: no-new-openspec-change-by-default
description: 用户默认不新开 OpenSpec change；除非明确要求，一律按工作流外实施推进
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 721bfd6f-2c01-4993-986b-87265cb78b7a
  modified: 2026-11-06T15:02:26.010Z
---

除非用户明确说「开 change / 走 OpenSpec」，否则**不要**创建新的 OpenSpec change
（`change_new.sh`）。直接改代码 + x86 编译回归 + 真机验证，完成后在
`openspec/tasks/YYYY-MM-DD-<kebab-name>.md` 按三节（构思 / 实现方式 / 结果）留记录。

**Why:** 用户 2026-11-06 明确说「不用新开 change；以后只要我不让你新开 change，
你就默认不用新开」。OpenSpec 的铁律（冻结指纹、evidence 契约、footprint）对
本项目当前节奏是纯开销，且代码先写再用 change 追认会因 footprint 为 0 报废。

**How to apply:** 收到任务先默认走工作流外实施；只有用户点名要 change、
或已有 active change 正在进行时才用 harness。工作流外任务仍须遵守：禁止自动
commit/push、禁止改 `scripts/`、文档与代码同步、任务记录必留。参见
[[planning-fingerprint-immutability]]（那个约束只在真的走 change 时生效）。
