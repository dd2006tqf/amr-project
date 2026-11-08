# Task 1 实现简报：NavigateSequence 桥接节点 + dispatcher Action 接线

工作目录：`/home/ubuntu/个人学习/amr_dispatcher`（已在 main 分支，直接提交）
完整计划：`docs/superpowers/plans/2026-10-11-tier1-nav-adapter-reservation-topology.md` 的 Task 1（含 7 个 Step，全部要执行）。本简报是执行上下文补充，冲突时以计划 Task 1 为准。

## 必读源码（动手前先读）

- `src/amr_dispatcher_ros/include/amr_dispatcher_ros/dispatcher_node.hpp`（全文件，230 行）
- `src/amr_dispatcher_ros/src/dispatcher_node.cpp` 关键段：on_configure(:24-127)、on_activate(:129-163)、ScheduleTick(:639-752)、TrackingErrorCb/CompleteMission(:762-826)、FailMission(:828-957)、PublishState(:1049-1070)
- `src/amr_dispatcher_interfaces/action/NavigateSequence.action`、`ExecuteMission.action`（已有定义，不许改 .action 文件）
- `src/amr_dispatcher_ros/CMakeLists.txt`（executable 注册模式，仿 dispatcher_node 段 :47-63）
- `src/amr_dispatcher_ros/src/path_tracker_node.cpp`（了解 `/tracking/error` 的字段与发布时机）

## 环境与验证命令

- 宿主无 ROS2。core 测试：`bash scripts/build.sh`（基线 461/461 全绿，不得回归）。
- ROS 层编译：`bash scripts/container_build.sh`（docker 镜像 `amr-dispatcher:jazzy` 已存在，约几分钟）。
- 冒烟脚本（若镜像支持 `ros2 action` CLI）：按计划 Task 1 Step 6 的 docker run 命令执行。
  若镜像缺 CLI 导致无法冒烟：**不要降低断言硬凑通过**，改为在报告中记录"降级为编译验证 + 手工冒烟步骤"，并把手工步骤写进脚本头部注释。
- 提交规范：commit message 末尾带
  `Co-Authored-By: Claude Code <noreply@anthropic.com>`

## 硬约束

1. `navigation_backend` 默认 `plan` 时，现有代码路径行为逐字节不变（现有测试/脚本不得受影响）。
2. `nav2_msgs` 必须 `find_package(... QUIET)` 条件编译：镜像里没有它时容器构建照样通过。
3. 不修改 `.action`/`.srv`/`.msg` 文件。
4. 不动 `amr_dispatcher_core` 与 `amr_dispatcher_bt`（本任务全在 `amr_dispatcher_ros` + scripts）。
5. 删除 hpp 中 `ExecuteActionTask` 声明时，确认无任何调用点（grep）。
6. 只做计划 Task 1 范围内的事，发现范围外问题记录到报告"concerns"，不要顺手改。

## 报告

完成后把结构化报告写到 `/tmp/amr-tier1-task1-report.md`，字段：
status（done/blocked）、commits（hash+subject）、tests（build.sh 结果 + container_build 结果 + 冒烟结果，附关键输出）、concerns（发现的问题/降级项）、self_review（自己复查的结论）。
