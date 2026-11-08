# Tier 1 执行台账

**状态：已收尾（2026-10-12）** —— 全部任务 + 终审修复完成，最终 `check_all.sh` ALL CHECKS PASSED，core 470/470。
Commits：`364f88b` → `5f75f1d` → `7057a3d` → `1e779e3` → `2e3577f` → `b139e46`（docs 计数同步）。
Follow-up F1-F7、F11-F12 见下，留待 Tier 2 计划处理。

计划：`2026-10-11-tier1-nav-adapter-reservation-topology.md`
工作区：直接在 main 上（用户 2026-10-11 明确授权，不建分支/worktree）
基线：`bash scripts/build.sh` 461/461 通过（2026-10-11）；终态 470/470（+9 新用例）

## 任务状态

| Task | 状态 | Commit | 审查 | 备注 |
|---|---|---|---|---|
| 1 NavigateSequence 桥接 + Action 接线 | **完成（APPROVE）** | `364f88b` + `5f75f1d` | R0 NEEDS_FIXES → R1 修复 7/7 → 复审 APPROVE | 遗留已由 Task 2 清（mission_dispatch_start_） |
| 2 派发预留路网锁 + WFG 数据上环 | **完成（APPROVE）** | `7057a3d` | 任务级审查 APPROVE | Follow-up F1-F3 见下 |
| 3 topology 发布 | **完成（APPROVE）** | `1e779e3` | 任务级审查 APPROVE 零必修 | Follow-up F5-F7 见下 |
| 最终整体审查（6992be6..1e779e3） | **完成（修复+复审 APPROVE）** | `2e3577f`（+124/-28） | opus 终审 With fixes → F8/F9/F10 修复 3/3 → 范围复审 APPROVE 零必修 | 470/470、容器 5 包、冒烟 PASSED；复审新 Minor→F12 |

## Follow-up / 已裁决项（终审后汇总）

- **F1**（Task 2 审查）长持锁启发式键值断层：`mission_reserved_at_` 按 mission_id 键，detector 按 `m.reservation_id`（逗号锁串）查，kLongReservationHold 结构性不触发。计划规格自相矛盾；单车下直接修通会误杀 >30s 正常任务。**Tier 1 后专修**（与 F8 一并改 detector：按 mission_id 追踪 + 仅在有其他车等待时判死锁）。
- **F2**（Task 2 审查）`reservation_denied` 10Hz 事件/WARN 刷屏，需节流退避（Important，Tier 1 后）。
- **F3**（Task 2 审查）人工 `ReleaseResourceCb` 不清 `mission_reserved_at_`（Minor，因 F1 未激活暂无影响）。
- **F5**（Task 3 审查）API.md 补 TopologyState.deadlock_detected 语义注释（仅表 WFG 环，启发式走 DispatcherState；审查确认 fault_supervisor 订前者→kPauseDispatch，语义差异有主无漏洞）。
- **F6**（Task 3 审查）锁 id 解析逻辑双份（topology_projection.cpp / traffic_planning.cpp）。
- **F7**（Task 3 审查）DEPLOYMENT.md:715 残留"453 个测试"。
- **F8**（终审 Important-1）unknown dropoff 判负分支漏 `FinishExecuteGoal` → ExecuteMission 客户端悬挂 + handle 泄漏。**本轮必修**（一行）。
- **F9**（终审 Important-2）`DetectCycles` 的 cycle_path 只含 mission id，不含资源 id → `deadlocked_nodes` 端到端恒空，Task 3 警戒柱断链。**本轮必修**（方案①：detector 记 waiter→resource→holder 交替路径 + 单测固化；与 F1 同域）。
- **F10**（终审 Important-3）`DetectTrafficDeadlockCb` 服务入口不填 WFG 快照，与 DeadlockTick 结论分叉。**本轮必修**（提取 `BuildDeadlockSnapshot()` helper 共用）。
- **F11**（终审 Minor）① bridge Nav2 feedback `fb->feedback.distance_remaining` 字段路径疑似错（#ifdef 内未编译过，Tier 2 Nav2 环境修）；② deactivated 期间 action server 仍受理 goal；③ `active_missions_mutex_` 加锁混用（单线程下安全，建议统一）；④ 排队/等待期 ExecuteMission 无 feedback。
- **F12**（F8-F10 复审 Minor）① 冒烟脚本 [OK] 实际 7 处、报告称 8 虚高 1；② `/detect_traffic_deadlock` 服务路径无测试；③ 同持有者资源去重只留序首进 deadlocked_nodes；④ conflict_station_ids 恒空（宜并 F6 一起收敛解析逻辑）；⑤ detector 无自环/多环专项测试（改前亦无）。

## Rulings（执行期裁决，均有据可查）

- R1: 工作区不隔离，直接 main 提交（用户选择）。
- R2: 基线实测 461 用例，README/check_all.sh 中"453"为过时数字，Task 3 统一修正（→468）。
- R3: 终审 F8/F9/F10 本轮必修；F1/F2/F3/F5/F6/F7/F11 记 Follow-up，Tier 2 计划时处理。

## 审查记录

- R0 Task1：NEEDS_FIXES（Critical 槽位锁死 + Important×4 + Minor×2）→ 报告 /tmp/amr-tier1-task1-review-r1.md
- R1 Task1：7/7 FIXED → 复审 APPROVE（观察：活动取消分支 Release 二次调用，幂等无影响）
- Task2：APPROVE + Follow-up（/tmp/amr-tier1-task2-report.md）
- Task3：APPROVE 零必修（/tmp/amr-tier1-task3-report.md）
- 终审（6992be6..1e779e3）：With fixes —— F8/F9/F10 必修，无 Critical
