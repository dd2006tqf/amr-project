# Task 2 实现简报：派发路径自动预留路网锁 + WFG 死锁数据上环

工作目录：`/home/ubuntu/个人学习/amr_dispatcher`（main 分支，直接提交）
完整计划：`docs/superpowers/plans/2026-10-11-tier1-nav-adapter-reservation-topology.md` 的 Task 2（设计决策 1-5 + Step 1-7，全部执行）。冲突时以计划 Task 2 为准。

## 前置依赖

Task 1 已完成（dispatcher_node.hpp/cpp 已含 action 接线成员）。你在 Task 1 改动后的代码基础上工作；计划 Task 2 Step 4 提到的四处 Release 清理点，行号可能已因 Task 1 漂移，**以 `grep -n "reservation_table_->Release" src/amr_dispatcher_ros/src/dispatcher_node.cpp` 实际结果为准**，同时清理 Task 1 的 `mission_dispatch_start_`（若该成员存在）。

## 必读源码

- `src/amr_dispatcher_core/include/amr_dispatcher_core/workflow/traffic_planning.hpp`（现有 BuildRouteLockIds 等纯函数）
- `src/amr_dispatcher_core/include/amr_dispatcher_core/catalog/station_catalog.hpp`（FindRoutePath 签名）
- `src/amr_dispatcher_core/include/amr_dispatcher_core/dispatcher/traffic_reservation.hpp`（Reserve/Release/Snapshot/BuildLockIds）
- `src/amr_dispatcher_core/include/amr_dispatcher_core/dispatcher/deadlock_detector.hpp`（DeadlockSnapshot 字段）
- `test/unit/test_traffic_planning.cpp`（既有测试风格）
- `src/amr_dispatcher_ros/src/dispatcher_node.cpp`：ScheduleTick、DeadlockTick、Complete/Fail/Cancel 的 Release 调用点

## 环境与验证

- `bash scripts/build.sh`（TDD：Step 1 先看到新测试编译失败/断言失败，Step 2 后全绿；基线 461 不得回归）
- `bash scripts/container_build.sh`（ROS 层编译）
- commit message 末尾带 `Co-Authored-By: Claude Code <noreply@anthropic.com>`

## 硬约束

1. 只改计划列出的文件（core 的 traffic_planning + test + dispatcher_node hpp/cpp + CMake 注册测试）。
2. core 不得 include rclcpp。
3. Peek→Reserve→Pop 顺序：预留失败时**任务必须留在队列**（不 Pop），发布 `reservation_denied` 事件（event_pub_ + event_log_ 双通道，抄既有事件代码模式），并 ReArm 后返回。
4. `FindRoutePath` 无路→空锁集→照旧无锁派发（保持现网行为）。
5. 人工 `ReserveResourceCb` 的 holder 不写入 `mission_reserved_at_`（避免长持锁误报）。
6. 发现范围外问题写报告 concerns，不顺手改。

## 报告

写到 `/tmp/amr-tier1-task2-report.md`，字段同 Task 1（status/commits/tests/concerns/self_review）。最终回复我时给：status、commits、验证结果（数字）、concerns。
