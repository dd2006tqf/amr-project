# Task 3 实现简报：发布 /dispatcher/topology_state

工作目录：`/home/ubuntu/个人学习/amr_dispatcher`（main 分支，直接提交）
完整计划：`docs/superpowers/plans/2026-10-11-tier1-nav-adapter-reservation-topology.md` 的 Task 3（设计决策 1-4 + Step 1-6，全部执行）。冲突时以计划 Task 3 为准。

## 前置依赖

Task 1、Task 2 已完成。dispatcher_node 已有 action 接线与预留成员；DeadlockTick 已填 allocation_graph。Task 3 的发布组包消费 Task 2 的 Snapshot 数据。

## 必读源码

- 计划 Task 3 设计决策 1-4
- `src/amr_dispatcher_interfaces/msg/TopologyState.msg`（字段唯一真源，不许改 msg）
- `src/amr_dispatcher_ros/src/dispatcher_visualizer_node.cpp` :168-227（locked_edges 期待 `route_edge:A__B` 格式、deadlocked_nodes 必须是站点 id）
- `src/amr_dispatcher_ros/src/safety_gate_node.cpp` :84-87（deadlock_detected 消费）
- `src/amr_dispatcher_core/CMakeLists.txt`（源文件列表 + enable_testing/test 注册模式 :101-140）
- `src/amr_dispatcher_ros/src/dispatcher_node.cpp`：on_configure/on_activate/on_deactivate/on_cleanup 的 publisher 生命周期、PublishState、DeadlockTick findings 存储点

## 环境与验证

- TDD：Step 1 新测试先红（编译失败），Step 2 实现后 `bash scripts/build.sh` 全绿（基线 461 + 新用例）
- `bash scripts/container_build.sh`
- `bash scripts/check_all.sh` 全链（注意：其中 heredoc 硬编码 test_cases 数字，按实际 `ctest --test-dir build/core --show-only=json-v1` 统计修正——基线实测 461，代码里写的是 453，README 同理若有该数字一并修正）
- commit message 末尾带 `Co-Authored-By: Claude Code <noreply@anthropic.com>`

## 硬约束

1. `TopologyProjection` 字段与 TopologyState.msg 逐字段对应；不改 .msg。
2. `deadlocked_nodes` 必须解析成站点 id（route_edge:A__B → A、B；route_node:X → X），mission id 不得混入——visualizer 拿它查 stations_ 画警戒柱，未知 id 会画到原点。
3. topic 名 `/dispatcher/topology_state`，与 safety_gate/visualizer 订阅一致。
4. publisher 生命周期与 state_pub_ 完全同步（create/activate/deactivate/cleanup 四处）。
5. 只做 Task 3 范围（core 新文件+测试+CMake、dispatcher 接线、check_all.sh 计数、README/docs 一行）。
6. 发现范围外问题写 concerns，不顺手改。

## 报告

写到 `/tmp/amr-tier1-task3-report.md`，字段同前。最终回复我时给：status、commits、验证结果（数字）、concerns。
