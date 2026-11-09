---
name: docs-cleanup-2026-10-13
description: 2026-10-13 docs/ 目录大清理——删除 4 篇过时/过程文档、迁移 1 篇 harness 内部文档、精简学习路线图、改名架构设计
metadata: 
  node_type: memory
  type: project
  originSessionId: 380b729c-a60f-4044-b44d-3d6bfc8083c9
  modified: 2026-10-13T15:57:05.888Z
---

2026-10-13 对 `docs/` 做了结构性清理（用户批准全部方案）：

- **删除**：`docker-buildx-优化方案.md`（QEMU 方案已被 2026-10-11 裸机交叉编译取代）、`项目评估.md`（自评报告无指导价值）、`ideas/fix-a2dp-l2cap-chan-offset.md`（已修复完毕）、`edge-telemetry-evolutions-1-2-implementation-plan.md`（Planner 底稿，内容已实施完毕）。
- **迁移**：`docs/ai/workflow-errors-ref.md` → `.ai-harness/docs/workflow-errors-ref.md`（harness 内部排错文档不属于项目 docs）。
- **改名**：`docs/架构设计.md` → `docs/架构设计-模块与代码结构.md`（与 `架构设计-五层权责契约.md` 区分），所有跨文件引用已同步更新，文档开头加了互链。
- **精简**：`docs/学习路线图.md` 从 914 行砍到 818 行（删 §8 应用场景五小节、§9 学习资源整节，结论章更新为当前事实）。
- **标注废弃**：`docs/交叉编译与开发板部署.md` 的 §4.1/§4.3 容器镜像、§8 `-j1` 警告、附录 D Dockerfile 全部标"历史参考，已废弃"，顶部加了当前主路径提示（sysroot 初始化 + vmlinux.h 两步即可）。

**第二批（同日 commit `877f314`，用户裁决"工作流外实施的不用记录"）**：
- **删除**：`docs/Phase2A-DNS-Service-Assurance-验证记录.md`（工作流外实施记录）、`docs/superpowers/specs/` 整个目录（4 份阶段设计稿）——我曾建议保留（理由：唯一记录+技能路径引用），用户否决并删除；引用已修复（交叉编译文档内联了 raw_syscall 事实，brainstorming SKILL 的设计输出路径改指 `openspec/tasks/`）。
- **移动**：`docs/ai/W3_calibration_specification.md` → `docs/W3参数校准规范.md`（用户正确指出它是 Assurance 领域规范、不是工作流文档，放 docs/ai 是错位）。
- **新增工作流规则（用户发起）**：不走 OpenSpec change 的任务必须在 `openspec/tasks/` 建 `YYYY-MM-DD-<kebab-name>.md`，按 README 三节模板记**构思/实现方式/结果**；规则写入 CLAUDE.md"规划与实现分离"节 + 工作流使用指南判定表。该目录不受 harness 约束但**不具备验收效力**。已验证 `openspec_cli.sh list` 对新目录无感知、exit 0。

**Why**: 用户的取舍标准是"工作流外的记录价值低、宁可删"，与我倾向保留唯一记录的判断相反——后续遇到类似"是否保留过程文档"问题，优先问用户而非默认保留。
**How to apply**: docs 清理类问题先列删除候选给用户裁决；工作流规则变更写 CLAUDE.md（scripts/ 仍禁止中途改）。

**相关**：[[edge-uplink-epoch-collision]]、[[arm64-build-environment]]、[[project-profile-cmake-migration]]
