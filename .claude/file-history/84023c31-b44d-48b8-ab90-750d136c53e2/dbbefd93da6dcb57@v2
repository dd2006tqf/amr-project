# Tier 1：导航适配 / 交通预约上环 / 拓扑状态发布 —— 实施计划

> REQUIRED SUB-SKILL: Use subagent-driven-development or executing-plans

**Goal:** 补齐 amr_dispatcher 三处"接口有了、环没闭上"的缺口：① NavigateSequence/ExecuteMission Action 真实接线（含 Nav2 薄适配）；② 派发路径自动预留路网锁并给 WFG 死锁检测喂真实数据；③ 发布 `/dispatcher/topology_state` 唤醒 safety_gate/visualizer 的死锁输入。

**Architecture:** 遵守项目既有纪律——业务决策放 `amr_dispatcher_core`（零 ROS、可 gtest），ROS 层只做薄接线。默认参数下现有行为逐字节不变（`navigation_backend:=plan` 走原 `/plan` 路径；预约失败时任务留在队列等价于今天"直接派发"之外的新增守卫仅在有路由锁时生效）。

**Tech Stack:** C++20（core）/ C++17（ros）、rclcpp_lifecycle、rclcpp_action、BehaviorTree.CPP v4（不新增树）、GoogleTest、docker 内 colcon 构建。

**Spec:** 本文件；背景见 `docs/TECHNICAL_SPECIFICATION.md`、`docs/DESIGN_DECISIONS.md` ADR-001。

## Global Constraints

- core 新增代码不 include 任何 rclcpp 头；ros 包编译标准 C++17。
- 不改 `/plan`、`/dispatcher/state`、`/dispatcher/events` 既有语义；新话题仅 `/dispatcher/topology_state`（已有 msg 定义）与 `/navigate_sequence`（已有 action 定义）。
- 每个 Task 独立可测、独立 commit；commit message 末尾带 `Co-Authored-By: Claude Code <noreply@anthropic.com>`。
- 验证命令（宿主）：`bash scripts/build.sh`（core 编译+ctest）；验证命令（ROS 层）：`bash scripts/container_build.sh`。
- `nav2_msgs` 在镜像中可能缺失：必须 `find_package(nav2_msgs QUIET)` 条件编译（见 Task 1 Step 6），保证容器构建永不因 Nav2 挂掉。
- 计划不覆盖 Tier 2/3（设施 TTL、充电策略、站点确认、事件查询、车队、诊断、REST 鉴权）——那些在 Tier 1 完成并核对代码后另写计划。

---

### Task 1: NavigateSequence 桥接节点 + dispatcher Action 接线

**Files:**
- Create `src/amr_dispatcher_ros/include/amr_dispatcher_ros/navigate_sequence_bridge_node.hpp`
- Create `src/amr_dispatcher_ros/src/navigate_sequence_bridge_node.cpp`
- Modify `src/amr_dispatcher_ros/CMakeLists.txt`（注册新可执行文件，nav2_msgs 条件依赖）
- Modify `src/amr_dispatcher_ros/include/amr_dispatcher_ros/dispatcher_node.hpp`（删除未用的 `ExecuteActionTask` 声明 :155，新增 action 客户端/服务端成员）
- Modify `src/amr_dispatcher_ros/src/dispatcher_node.cpp`（on_configure 注册 ExecuteMission server + 可选 NavigateSequence 客户端；派发分支；结果回流）
- Create `scripts/check_navigate_sequence_bridge.sh`（容器内端到端冒烟）

**Interfaces:**
- Consumes: `NavigateSequence.action`（waypoints/station_names/loop）、`ExecuteMission.action`（已有定义）、`/plan`、`/tracking/error`、Nav2 `navigate_to_pose`（可选）
- Produces: `/navigate_sequence` action server（bridge）、`ExecuteMission` action server（dispatcher）、参数 `navigation_backend`（`plan`|`action`，默认 `plan`）

**设计决策（写死，不留开放项）：**
1. bridge 的 fallback 模式（`use_nav2:=false`，默认）：goal 的 `waypoints` 直接转 `nav_msgs/Path` 发 `/plan`（复用既有 path_tracker 执行链），订阅 `/tracking/error`，`arrived==true` 即该 waypoint 到达；`loop:=true` 到达后从头重发，直到 cancel。
2. bridge 的 Nav2 模式（`use_nav2:=true` 且编译期 `AMR_HAVE_NAV2_MSGS`）：按 `waypoints` 顺序逐个向 `/navigate_to_pose` 发 goal，feedback 映射为 `current_index/total/current_station/state_code=2/percent=index/total*100`，单点失败→整 goal abort；cancel 级联 `async_cancel_all_goals`。
3. dispatcher `navigation_backend:=action` 时：派发不发 `/plan`，改为向 `/navigate_sequence` 发单点 goal（waypoint=planner 算出的终点，station_names=[dropoff]）；feedback 更新 `mission_last_progress_`（喂死锁 idle 启发式）；result success→`CompleteMission`，failure→`FailMission`；`CancelOrderCb`/`DeadlockTick`/`FailMission` 取消路径调 `async_cancel_goal`。`plan` 模式（默认）代码路径与当前完全一致。
4. `ExecuteMission` server：`HandleGoal` 校验非空 mission_id + 队列不重复 + `required_resources` 可预占（`reservation_table_->Reserve` 失败即 REJECT）；`HandleAccepted` 只做"Mission 入队 + 存 goal handle 到 `execute_goal_handles_[mission_id]`"，**不起线程**（删除 `ExecuteActionTask` 声明）；`CompleteMission`/`FailMission` 内查表 finish goal（success→succeed、fail→abort error_code=1、用户取消→canceled）；feedback 在 `PublishState`（500ms）里按 dispatch 起点直线距离算 `percent_complete`。

- [ ] **Step 1: 写 bridge 节点**（hpp+cpp，约 350 行）。要点：继承 `rclcpp::Node`（bridge 不需要 lifecycle）；参数 `use_nav2`、`nav2_action_name`（默认 `/navigate_to_pose`）、`plan_topic`（默认 `/plan`）；fallback 分支的 Path 转换用 `goal.waypoints` 逐点填 `PoseStamped`（frame_id="map"，orientation 由相邻点 atan2 得 yaw；单点时 yaw=0）；Nav2 分支包在 `#ifdef AMR_HAVE_NAV2_MSGS` 内，未编译时 `use_nav2:=true` 在 on_configure 打 FATAL 并降级 fallback。
- [ ] **Step 2: CMake 注册**：仿照 `dispatcher_node` 段落（CMakeLists.txt :47-63）新增 `navigate_sequence_bridge_node` executable；`find_package(nav2_msgs QUIET)` 成功则 `target_compile_definitions(... AMR_HAVE_NAV2_MSGS)` 并 `ament_target_dependencies(nav2_msgs)`。
- [ ] **Step 3: 容器编译验证 A**：`bash scripts/container_build.sh`，确认新节点编过、无警告级错误。
- [ ] **Step 4: dispatcher 接线**
  - hpp：删 `ExecuteActionTask` 声明；加成员 `rclcpp_action::Server<ExecuteMission>::SharedPtr execute_action_server_`、`rclcpp_action::Client<NavigateSequence>::SharedPtr nav_client_`、`std::unordered_map<std::string, rclcpp_action::ServerGoalHandle<ExecuteMission>::SharedPtr> execute_goal_handles_`、`std::unordered_map<std::string, amr_dispatcher_core::path_tracking::Pose2D> mission_dispatch_start_`、`std::string navigation_backend_{"plan"}`。
  - cpp `on_configure`：读参数 `navigation_backend`；`create_server<ExecuteMission>("/execute_mission", HandleGoal, HandleCancel, HandleAccepted)`；`action` 模式下 `create_client<NavigateSequence>("/navigate_sequence")`。
  - `ScheduleTick`：`navigation_backend_=="action"` 时走分支（发 goal、存 `pending_nav_mission_id_`、不起 `plan_pub_`、`tracking_mission_id_` 仍置位占住单车槽）；否则原路径不动。goal 的 waypoint 仍由 `planner_->Plan` 的最后一点给出（起点解析逻辑 :672-713 不动）。
  - 新增 `NavResultCb`/`NavFeedbackCb`：success→`CompleteMission(id,"navigate_sequence succeeded")`；failure→`FailMission(id, result.msg)`；feedback→`mission_last_progress_[id]=now()`。
  - `CompleteMission`/`FailMission`/`CancelOrderCb` 活动取消分支： finish/取消 `execute_goal_handles_` 中的 handle；`action` 模式下 `async_cancel_goal` 清 `pending_nav_mission_id_`。
  - `PublishState`：对 `execute_goal_handles_` 中仍关联 active mission 的 handle 发 feedback（state_code=1、percent 由 `mission_dispatch_start_` 直线距离算）。
- [ ] **Step 5: 写失败冒烟脚本** `scripts/check_navigate_sequence_bridge.sh`：容器内 `ros2 run` 起 bridge（fallback），`ros2 action send_goal /navigate_sequence` 发 2 点 goal，后台 `ros2 topic pub -1 /tracking/error` 伪造 arrived（需两次，覆盖第 1、2 点），断言 goal SUCCEEDED；再发 `loop:=true` goal 断言持续运行后 cancel 成功。脚本用超时+grep 结果文本，任何一步失败退出非 0。
- [ ] **Step 6: 容器编译 + 冒烟验证 B**：`bash scripts/container_build.sh` 通过后，在容器内跑 `scripts/check_navigate_sequence_bridge.sh`（`docker run --rm -v $PWD:/workspace -w /workspace amr-dispatcher:jazzy bash -c "source /opt/ros/jazzy/setup.bash && source install/setup.bash && bash scripts/check_navigate_sequence_bridge.sh"`）。
- [ ] **Step 7: Commit** `feat(ros): NavigateSequence 桥接节点与 ExecuteMission/NavigateSequence Action 接线`

---

### Task 2: 派发路径自动预留路网锁 + WFG 死锁数据上环

**Files:**
- Modify `src/amr_dispatcher_core/include/amr_dispatcher_core/workflow/traffic_planning.hpp`（新增纯函数声明）
- Modify `src/amr_dispatcher_core/src/workflow/traffic_planning.cpp`（实现）
- Modify `test/unit/test_traffic_planning.cpp`（新增用例）
- Modify `src/amr_dispatcher_ros/src/dispatcher_node.cpp`（ScheduleTick 预留、DeadlockTick 填快照、终态清理）
- Modify `src/amr_dispatcher_ros/include/amr_dispatcher_ros/dispatcher_node.hpp`（新成员）

**Interfaces:**
- Consumes: `StationCatalog::FindRoutePath`、`workflow::BuildRouteLockIds`、`ResourceTable::Reserve/Release/Snapshot`、`DeadlockSnapshot.allocation_graph/reserved_resource_ids/reservation_acquired_at`
- Produces: 任务级 `mission.reservation_id` 写入；`/dispatcher/events` 新事件 `reservation_denied`；死锁 `waiting_for`/`held_by` 数据流

**设计决策：**
1. **先 Peek 再预留**：runtime 门控通过后不立即 Pop，先 `queue_->Peek()`，为候选任务规划路由锁并 `Reserve(m.id, locks)`；成功才 `PopNext()` 继续派发（节点单线程，Peek 与 Pop 之间无竞态）。失败则任务留在队列、发布 `reservation_denied` 事件、`ReArmQueueStartRequest()` 本 tick 返回——既不丢任务也不违反封路。
2. **路由锁规划** `PlanMissionRouteLocks(catalog, from, to)`：`from`/`to` 均非空→`FindRoutePath` 得站点序列→`BuildRouteLockIds(path, true)`（含路口锁）；`from` 空→只锁目的站节点锁 `RouteNodeLockId(to)`；`FindRoutePath` 无路（nullopt）→返回空 vector，派发照旧无锁（保持现网行为）。
3. **WFG 喂数据**（`DeadlockTick` :964-968 处）：`allocation_graph.held_by` ← `reservation_table_->Snapshot()`（resource→owner）；`allocation_graph.waiting_for` ← `pending_wait_`（Peek 预留失败的 mission→锁列表）；`reserved_resource_ids` ← Snapshot 全部 resource id；`reservation_acquired_at` ← `mission_reserved_at_`。单车模型下不会构造出假环（等待者在队列、持有者不新增等待），多车接入后数据直接可用。
4. **清理**：`CompleteMission`/`FailMission`/`CancelOrderCb`/`DeadlockTick` 取消分支里凡调 `Release(mission_id)` 处，同步 `mission_reserved_at_.erase`、`pending_wait_.erase`、`mission_dispatch...`（Task 1 的 `mission_dispatch_start_` 同点清理）。预留成功时把 `m.reservation_id` 写成锁列表以 `,` 连接的字符串。
5. 长持锁启发式（`reservation_hold_threshold`）从此获得真实 `reservation_acquired_at`，只记任务持有（人工 `ReserveResourceCb` 不记，避免误报）。

- [ ] **Step 1: 写失败测试** `test/unit/test_traffic_planning.cpp` 追加 `TEST(TrafficPlanningTest, PlanMissionRouteLocks...)` 三用例：① 两站有边→返回含 `route_edge:A__B` 与 `route_node:` 的锁集；② from 空→仅含 `route_node:B`；③ 无路（目录断开）→空 vector。先跑 `bash scripts/build.sh` 确认编译失败（函数未定义）。
- [ ] **Step 2: 实现纯函数**：`traffic_planning.hpp` 加
  `std::vector<std::string> PlanMissionRouteLocks(const amr_dispatcher_core::catalog::StationCatalog& catalog, const std::string& from_station, const std::string& to_station);`
  （cpp 实现按设计决策 2；hpp 顶部前向声明 `namespace amr_dispatcher_core::catalog { struct StationCatalog; }`，头文件放不下完整定义时改 include station_catalog.hpp——二选一，以能编过为准，倾向 include）。跑 `bash scripts/build.sh` 全绿。
- [ ] **Step 3: dispatcher 派发接入**：hpp 加 `std::unordered_map<std::string, std::vector<std::string>> pending_wait_;`、`std::unordered_map<std::string, std::chrono::steady_clock::time_point> mission_reserved_at_;`。cpp `ScheduleTick` 门控通过后、`queue_->PopNext()` 前按设计决策 1 插入预留块；预留成功后 `m.reservation_id` 回填（Pop 后写入 `active_missions_[m.id]` 前的 `m` 上）。`reservation_denied` 事件走既有 `event_pub_` + `event_log_` 双通道（抄 :689-698 的模式）。
- [ ] **Step 4: 终态清理**：在四处 `Release` 调用点旁加 erase（CompleteMission :798、FailMission :851、CancelOrderCb :306、DeadlockTick :1000）。
- [ ] **Step 5: DeadlockTick 填快照**：按设计决策 3 改 :964-968。
- [ ] **Step 6: 验证**：`bash scripts/build.sh` 全绿（含新用例）；`bash scripts/container_build.sh` 编过。
- [ ] **Step 7: Commit** `feat(dispatch): 派发路径自动预留路由锁并喂 WFG 死锁快照`

---

### Task 3: 发布 /dispatcher/topology_state（纯函数投影 + 生命周期接线）

**Files:**
- Create `src/amr_dispatcher_core/include/amr_dispatcher_core/dispatcher/topology_projection.hpp`
- Create `src/amr_dispatcher_core/src/dispatcher/topology_projection.cpp`
- Create `test/unit/test_topology_projection.cpp`
- Modify `src/amr_dispatcher_core/CMakeLists.txt`（加源文件 + 测试注册，仿 :101-140 的 `test_*` 列表）
- Modify `src/amr_dispatcher_ros/src/dispatcher_node.cpp`（create/activate/publish/reset）
- Modify `src/amr_dispatcher_ros/include/amr_dispatcher_ros/dispatcher_node.hpp`（`last_deadlock_station_ids_`、`last_deadlock_findings_` 存储）
- Modify `scripts/check_all.sh`（heredoc 中硬编码的 suite/case 计数按实际 `ctest --show-only=json-v1` 更新）

**Interfaces:**
- Consumes: `ResourceTable::Snapshot`、active mission 集合、`DeadlockFinding.cycle_path`
- Produces: `TopologyState.msg`（active_edges / locked_edges / edge_owners / deadlocked_nodes / deadlock_detected）

**设计决策（对齐 visualizer 消费方式）：**
1. 纯函数 `struct TopologyProjection { std::vector<std::string> active_edges, locked_edges, edge_owners, deadlocked_nodes; bool deadlock_detected; }` + `TopologyProjection BuildTopologyProjection(const std::vector<std::pair<std::string,std::string>>& resource_owner_snapshot, const std::unordered_set<std::string>& active_mission_ids, const std::vector<std::vector<std::string>>& deadlock_cycle_paths);`
2. `locked_edges`/`edge_owners` = Snapshot 全量按 resource id 排序（Snapshot 已排序，直接透传）；`active_edges` = Snapshot 中 owner ∈ active_mission_ids 且 id 带 `route_edge:` 前缀的 resource；`deadlocked_nodes` = 遍历 cycle_path 中 `route_edge:A__B`（取 A、B）与 `route_node:X`（取 X）项，解析出站点 id 去重——**必须是站点 id**，因为 visualizer 拿它查 `stations_` 地图画警戒柱（dispatcher_visualizer_node.cpp:206-211）；`deadlock_detected` = cycle_path 非空。
3. 发布接线：`on_configure` create `topology_pub_`（话题 `/dispatcher/topology_state`，depth 10），`on_activate/on_deactivate/on_cleanup` 与 `state_pub_` 同步增删；`PublishState()`（500ms）尾部组包发布；`DeadlockTick` 每次把 `findings` 的 cycle_path 存 `last_deadlock_findings_`（无死锁时清空）。
4. 消费方立刻生效：safety_gate `deadlock_active_`（safety_gate_node.cpp:84-87）与 visualizer 绿色锁边/红色警戒柱从"永远空"变真实。

- [ ] **Step 1: 写失败测试** `test/unit/test_topology_projection.cpp`：① 空输入→全空 + deadlock_detected=false；② 快照含 `route_edge:A__B`（owner=m1，active={m1}）与人工封路 owner=`traffic_block:x`→locked_edges 两条、active_edges 仅 m1 那条；③ cycle_path `["m1","route_edge:A__B","m2","route_node:C","m1"]`→deadlocked_nodes=={A,B,C} 去重有序；④ cycle_path 只有 mission id→deadlocked_nodes 空（不画到原点的假警戒）。CMake 注册后 `bash scripts/build.sh` 确认编译失败。
- [ ] **Step 2: 实现** `topology_projection.hpp/cpp`（严格按设计决策 1-2；解析逻辑复用 `traffic_planning::ParseRouteLock` 的等价规则，本文件内自带 `route_edge:`/`route_node:` 前缀解析，不 include workflow 层——保持 dispatcher 域内聚）。`bash scripts/build.sh` 全绿。
- [ ] **Step 3: dispatcher 接线**：按设计决策 3 改 hpp/cpp。
- [ ] **Step 4: 验证**：`bash scripts/build.sh`；`bash scripts/container_build.sh`；`bash scripts/check_all.sh` 跑全链并按实际用例数更新 heredoc 计数。
- [ ] **Step 5: 文档**：`README.md` 接口段（若有 topics 清单）与 `docs/API.md` 补 `/dispatcher/topology_state` 一行；`src/amr_dispatcher_bt` 不涉及。
- [ ] **Step 6: Commit** `feat(ros): 发布拓扑状态话题接通安全门与可视化死锁输入`

---

## Self-Review 结论

- 覆盖：Tier 1 三个条目各对应一个 Task；Nav2 适配含 fallback 保底（Task 1 设计 1-2）；交通预约上环（Task 2）；topology 发布（Task 3）。
- 无占位符：所有 topic/action/参数/文件路径/行号均来自本次实读源码。
- 类型一致性：`TopologyProjection` 字段与 `TopologyState.msg` 逐字段对应；`PlanMissionRouteLocks` 返回值与 `ResourceTable::Reserve` 的 `vector<string>` 入参一致；Task 1 的 `mission_dispatch_start_` 与 Task 2 清理点共用四处 Release 位置。
- 风险备注：容器冒烟脚本（Task 1 Step 5）依赖镜像内 DDS 单机自发现；若镜像缺 `ros2 action` CLI，则降级为"编译验证 + 记录手工冒烟步骤"，不阻塞合入。
