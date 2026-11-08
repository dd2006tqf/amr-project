# 全仓审查修复（安全链 + 中优先级 + 工程化）Implementation Plan

> REQUIRED SUB-SKILL: Use subagent-driven-development or executing-plans

**Goal:** 修复全仓审查发现的安全链 4 个高危、core/ROS 全部中优先级问题、以及 CI/文档/仓库卫生工程化缺口。
**Architecture:** 保持现有分层（core 零 ROS / ros 集成 / bt 编排 / tools），在既有接口上补齐语义，不引入新子系统。
**Tech Stack:** C++17、ROS 2 Jazzy、GoogleTest、bash CI、GitHub Actions。
**Spec:** 本文件；来源为 2026-10-19 全仓审查报告（4 路并行审查的发现清单）。

## Global Constraints

- 所有行为修复走 TDD：先写失败测试，确认失败，再实现，确认通过。
- 每个工作流结束必须 `ctest`（core：`ctest --test-dir build/core`；BT：core 里跑 bt ctest；ROS：colcon build）全绿，输出证据。
- 不触碰孤儿接口的删除决策（docking/facility BT、孤儿 yaml、bringup.launch 等留待后续讨论），但本计划会把 config yaml `dispatcher/chassis/safety` 三个接入 launch（用户已确认方向：让 config 生效）。
- 代码注释与现有风格一致（中文注释密度对齐周边）。
- 提交按工作流分批，消息风格对齐历史（`fix(...)`/`feat(...)` 中文摘要），结尾加 Co-Authored-By。

---

## Workstream A — core 安全与正确性（src/amr_dispatcher_core + test/unit）

### Task A1: CmdVelGate 事件 TTL + NaN 归零

**Files:** `include/amr_dispatcher_core/safety/cmd_vel_gate.hpp`、`src/safety/cmd_vel_gate.cpp`、`test/unit/test_cmd_vel_gate.cpp`
- [ ] 测试先行：(a) push kCritical 事件后，`Evaluate(cmd, now)` 在事件 stamp + ttl 内 blocked、超过 ttl 后放行；(b) `Scale(NaN)` 返回 0；`Scale(Inf)` 返回 ±max；(c) 现有 blocked/pass 行为不回归。
- [ ] 实现：`SafetyEvent` 沿用 `stamp`；`Evaluate` 增加 `now` 参数（重载或默认实参），过滤 `now - e.stamp > ttl`（`CmdVelGateConfig` 增 `event_ttl`，默认 2000ms）；`Scale` 先 `std::isfinite` 检查，非有限返回 0。

### Task A2: LinkQualityMonitor 真滑动窗口 + 恢复信号边沿化 + window_size clamp

**Files:** `src/chassis/link_quality.cpp`、`include/.../chassis/link_quality.hpp`、`test/unit/test_link_quality.cpp`
- [ ] 测试先行：(a) 长期健康（>window 全成功）后突发连续丢包，loss_rate 快速上升到 degraded/critical（累计语义下测不出）；(b) 丢包消失且窗口滑过后恢复 kHealthy（无需 Reset）；(c) `window_size=0` 构造不崩溃且行为等价 window=1；(d) 恢复信号只在首次达阈值触发一次（连续成功 100 帧 `ConsumeRecoverySignal` 只 true 一次）。
- [ ] 实现：`RecomputeLocked` 遍历 `window_` ring（用 `window_filled_`）统计成功率/时延；构造 clamp `window_size>=1`；恢复信号改为进入条件首次满足置 bool、`ConsumeRecoverySignal` 消费后清零、`Reset`/降级时清除。

### Task A3: FaultSupervisor 上报时效性

**Files:** `include/.../safety/fault_supervisor.hpp`、`src/safety/fault_supervisor.cpp`、`test/unit/test_transport_and_diagnostics.cpp`
- [ ] 测试先行：(a) 某源上报健康后停止刷新超过 `staleness_timeout`（默认 2000ms），`Tick` 输出降级/该源按不可信处理；(b) 持续上报的源不受影响；(c) 从未上报过的源不误判（保持现状）。
- [ ] 实现：`UpdateMultiSourceHealth` 记录 `last_report_time_`（按源）；`Tick(now)` 对超时源按 critical/kUnknown 仲裁（沿用现有优先级语义）。

### Task A4: AsyncLogger 数据竞争与丢弃计数

**Files:** `include/.../logging/async_logger.hpp`、`src/logging/async_logger.cpp`、`test/unit/test_logger.cpp`
- [ ] 测试先行：(a) 多线程（≥4 生产者 × 10k 条）+ 周期 `Flush()` 压测，ASan/TSan 可选但逻辑上无竞争；(b) 队列溢出时 `dropped_count()` 精确计数；(c) `Flush()` 返回后文件内容包含 Flush 调用前所有已 Log 记录（含已出队未写盘的）。
- [ ] 实现：写盘移入 `queue_mutex_` 保护（writer pop+write 同持锁），或给 `stream_` 单独 mutex；`Flush` 等待 `queue_.empty()` 且 writer 不在写（`writing_` 标志 + cv）；`std::atomic<uint64_t> dropped_count_`。

### Task A5: Serial/UDP 后端部分写与 EAGAIN 语义

**Files:** `src/chassis/serial_backend.cpp`、`src/chassis/udp_backend.cpp`、对应测试
- [ ] 测试先行：serial 用 PTY（仓库已有 PTY 思路：scripts/mock_physical_chassis.py；core 测试如无 PTY helper 则新建小 helper）验证短写被循环写完；EAGAIN/ would-block 返回 false。
- [ ] 实现：`WriteCommand` 部分写循环重试至写完或错误；EAGAIN/EWOULDBLOCK 返回 false（计入 loss）；UDP 同理。

### Task A6: FacilityReservation 先查后放（失败无副作用）

**Files:** `src/facility/facility_reservation.cpp`、`test/unit/test_facility_reservation.cpp`
- [ ] 测试先行：holder 已持有资源 A，尝试预约被他人独占的 B 失败时，A 仍归 holder。
- [ ] 实现：把 `release_existing_for_holder` 移到 enabled/exclusive 冲突检查全部通过之后。

### Task A7: TrafficReservation 过期回收 + newly_reserved 去重

**Files:** `src/dispatcher/traffic_reservation.cpp`、`test/unit/test_traffic_reservation.cpp`
- [ ] 测试先行：(a) ReserveTimeSpace 过期条目在下次 Reserve/Release 时被 prune（表大小不增长）；(b) 同 mission 重复 Reserve 已持有锁不进 newly_reserved；(c) id 列表含重复不重复计入。
- [ ] 实现：Reserve/Release 入口 prune `end < now`；写入前过滤 `existing->second == mission_id` 与列表内重复。

### Task A8: DeadlockDetector kOverCapacity + RecoveryPolicy 不误伤

**Files:** `include/.../dispatcher/deadlock_detector.hpp`、`src/dispatcher/deadlock_detector.cpp`、`src/dispatcher/recovery_policy.cpp`、`test/unit/test_deadlock_detector.cpp`、`test/unit/test_recovery_policy.cpp`
- [ ] 测试先行：任务数超限 finding 的 kind 为 `kOverCapacity`；recovery 对 kOverCapacity 不 pause 活跃任务（转为暂缓派发/hold 类动作，复用现有动作枚举中最贴近者），kMutualBlock 仍 pause；更新 test_deadlock_detector.cpp:119-127 旧断言。
- [ ] 实现：新枚举值 + 判定处改 kind + recovery 分支。

### Task A9: ChassisPacketStream 溢出语义统一

**Files:** `src/chassis/chassis_packet_stream.cpp`、`test/unit/test_chassis_packet_stream.cpp`
- [ ] 测试先行：(a) 文本流塞入超长无 `\n` 行触发溢出后，后续正常行仍能解出包（自动恢复），`Overflowed()` 为事件标志可 Clear；(b) 二进制流溢出后后续帧能继续解析。
- [ ] 实现：文本流遇 `\n` 丢弃坏行继续；溢出只置 flag 不永久哑掉；二进制流保持可解析并在成功解帧时行为一致（flag 语义统一为"曾发生溢出"事件位）。

### Task A10: 路由锁 ID 单一实现（消除 dispatcher/workflow 重复）

**Files:** `include/.../dispatcher/traffic_reservation.hpp`（或新 `route_lock_id.hpp`）、`src/dispatcher/traffic_reservation.cpp`、`src/workflow/traffic_planning.cpp`、`test/unit/test_traffic_planning.cpp`
- [ ] 测试先行：跨命名空间 round-trip：workflow 生成的 lock id 能被 dispatcher `ParseRouteEdge`/`IsTrafficBlockOwner` 正确解析，反之亦然。
- [ ] 实现：`RouteEdgeId/ParseRouteEdge/BuildLockIds` 保留于 dispatcher（或提公共头），workflow 的 `RouteLockId/ParseRouteLock/BuildRouteLockIds` 改为薄封装调用同一实现；删除逐行重复体。

**A 收口：** `cmake --build build/core && ctest --test-dir build/core` 全绿（原 470 + 新增用例）。

---

## Workstream B — ROS 集成层（src/amr_dispatcher_ros、launch、config）【依赖 A 的 core API】

### Task B1: safety_gate 零速兜底 + 心跳分源

**Files:** `src/safety_gate_node.cpp`、`include/amr_dispatcher_ros/safety_gate_node.hpp`、`src/chassis_driver_node.cpp`、`src/path_tracker_node.cpp`、`launch/full_system.launch.py`
- [ ] 测试先行（l2 层暂无执行框架，用 `ros2 topic echo`/脚本断言不可行则以代码评审 + 手动冒烟记录；核心可测逻辑抽纯函数单测：staleness→zero 判定）。
- [ ] 实现：
  - safety_gate 增 50Hz 发布定时器：estop/bumper/critical → 发零速；`/cmd_vel_raw` 超过 `heartbeat_timeout_ms` 未收到 → 周期发零速；正常时发最近一次 Evaluate 结果（保持现有 Evaluate-on-raw 路径）。
  - 心跳分源：chassis_driver 发 `/safety/heartbeat/chassis_driver`，path_tracker 发 `/safety/heartbeat/path_tracker`（grep 更新所有引用旧 `/safety/heartbeat` 的脚本/文档）；safety_gate 两个独立订阅回调分别 Feed 对应源。
  - path_tracker 心跳无条件周期发布（移到空闲早退之前）。
  - chassis_driver `ReadPollLoop` 增指令年龄检查，超时主动写零速帧。
  - `health_rate_hz` 默认 2.0→5.0；launch 描述与默认值同步（full_system L24-30、L81 `heartbeat_timeout_ms` 保持 500，2Hz→5Hz 后不再处于误报边界）。

### Task B2: on_cleanup 收口在途 action goal

**Files:** `src/dispatcher_node.cpp`
- [ ] 实现：`on_cleanup` 中遍历 `execute_goal_handles_` 逐个 `abort()`（含释放其 reservation），再 clear；补注释。手动冒烟路径记录于提交说明。

### Task B3: navigate bridge 单航点 yaw + 接入 full_system

**Files:** `src/navigate_sequence_bridge_node.cpp`、`launch/full_system.launch.py`
- [ ] 测试先行：bridge 预处理函数抽纯函数（或直接对现有结构），单航点保留原始四元数；多航点才覆盖 orientation。
- [ ] 实现：`if (n > 1)` 分支内才重写 orientation；full_system.launch.py 启动清单加入 `navigate_sequence_bridge_node`。

### Task B4: mission id/sequence 统一

**Files:** `src/dispatcher_node.cpp`（Enqueue ~L469、Preempt ~L562、dock ~L1491、SubmitOrder ~L289）
- [ ] 测试先行：若可抽构造函数则单测 id/sequence 一致；否则统一三处为 `auto n = next_sequence_++; m.id = "m_" + std::to_string(n); m.sequence = n;` 并自查。

### Task B5: /v2 四个桩真实现

**Files:** `src/dispatcher_node.cpp`
- [ ] `estimate_mission_cost`：用 StationCatalog 站点坐标算直线距离和 + 航点数（2 站 1 航段），调用处传 catalog。
- [ ] `list_stations`：从 `catalog_reloader_->catalog()` 投影，不再硬编码。
- [ ] `validate_site_config`：真实 LoadFile `stations_file`（失败则 success=false + reason），检查 dispatcher 参数（queue_capacity>0 等最小集合），checked_files 填真实路径。
- [ ] `get_operator_snapshot`：接入 `workflow/operator_snapshot.hpp` 投影或至少输出 queue_size+active_mission_id+paused 计数的真实值。
- [ ] 各自补 handler 级单测不可行（ROS 节点），以手动 `ros2 service call` 冒烟 + 提交说明记录。

### Task B6: 参数热重载

**Files:** `src/dispatcher_node.cpp`、`include/amr_dispatcher_ros/dispatcher_node.hpp`
- [ ] 实现：`add_on_set_parameters_callback`（赋值 `param_callback_handle_`）：支持 `comparator`（调 `queue_->SetComparator` 映射）、`schedule_rate_hz`、`deadlock_check_rate_hz`、`max_active_missions`；回调内校验范围非法即 reject；定时器周期更新用 mutex 或原子读。README:64 宣称随之变为真实。

### Task B7: config 三件套接入 launch + 安装 share

**Files:** `config/dispatcher.yaml`、`config/chassis.yaml`、`config/safety.yaml`（转为 ROS param 文件格式）、`launch/full_system.launch.py`、`launch/core_only.launch.py`、`src/amr_dispatcher_ros/CMakeLists.txt`
- [ ] 实现：yaml 改为按节点名分节的参数文件（值取当前 launch 内联值，保证行为不变）；launch 改 `parameters=[PathJoinSubShareDirectory('amr_dispatcher_ros','config','xxx.yaml'), {...内联覆盖}]`；`stations_file` 用 share 绝对路径（修 cwd 依赖）；CMake `install(DIRECTORY config launch ...)`。
- [ ] `core_only.launch.py` 补 configure/activate 生命周期事件（修"永远 unconfigured"）。

### Task B8: DeadlockTick kRetry 写 map 前判存在

**Files:** `src/dispatcher_node.cpp`（~L1650）
- [ ] 实现：写 `mission_attempts_`/`mission_last_progress_` 前确认 mission 在 `active_missions_` 或队列中，否则跳过。

**B 收口：** `colcon build --packages-select amr_dispatcher_interfaces amr_dispatcher_core amr_dispatcher_ros amr_dispatcher_bt amr_dispatcher_tools` 全绿；`ros2 launch` full_system 冒烟（节点全起、action server 在）。

---

## Workstream C — 工程化（scripts、CI、docs、卫生）【与 B 并行，最终验证串行】

### Task C1: check_all.sh 诚实化 + 补 BT

**Files:** `scripts/check_all.sh`
- [ ] 实现：新增步骤跑 BT ctest（构建 bt 包后 `ctest`）；`test_report_matrix.json` 中 L2_node_contracts 改 `SKIPPED`（注明未接入执行框架）；删除硬编码 PASSED；BT 条目由实际 ctest 结果生成；verdict 基于实际跑过的阶段。
- [ ] `test_report_matrix.json` 加入 `.gitignore` 并 `git rm --cached`。

### Task C2: CI 收敛

**Files:** `.github/workflows/ci.yml`
- [ ] 实现：可执行校验清单补 `navigate_sequence_bridge_node`（10 个）；"453 tests"注释更新为动态/移除写死数字；新增 job 跑 `./scripts/check_all.sh`（或至少 3 个 chaos 脚本 + benchmark）；`actions/checkout@v3`→`v4`。

### Task C3: README 修正

**Files:** `README.md`
- [ ] L14 保留 470/470（与实测一致）；L94 "139 GTest" → 与 check_all 实际一致（470）；L95 benchmark 数字与 `test_report_matrix.json` 统一；L41 `cmd_vel_safety_gate_node` → `safety_gate_node`；L64 热重载宣称在 B6 后保留（现在为真）。

### Task C4: USAGE.md 修订

**Files:** `USAGE.md`
- [ ] 目录树补 `amr_dispatcher_bt`；测试套件/用例数改为实测（52 套件/470 + BT 36）；`run_demo` 描述同步。

### Task C5: docs/API.md 补全

**Files:** `docs/API.md`
- [ ] 增补 12 个 `/v2/*` 服务表（名称/请求/响应/实现状态——桩的标注"基础实现"）、6 个 legacy 服务保留、`ExecuteMission.action` 章节、REST 缺失端点（`/api/operator/snapshot`、`/api/operator/estop`、`/api/system/ready`、`/api/workflows/cancel`）。

### Task C6: 文档措辞与仓库卫生

**Files:** `docs/ARCHITECTURE.md`、`docs/EXTENSION_ROADMAP.md`、`.gitignore`、scripts、`docker/Dockerfile.jazzy`
- [ ] "无锁队列"→"异步低阻塞队列"（A4 之后如实描述）。
- [ ] `.gitignore` 增 `docs/superpowers/`、`test_report_matrix.json`；`git rm --cached test_report_matrix.json`。
- [ ] 删除空目录：`docs/diagrams/`、`third_party/`、`src/amr_dispatcher_tools/launch`、`src/amr_dispatcher_tools/src/cli`、`src/amr_dispatcher_tools/src/rest`、`src/amr_dispatcher_ros/src/ros2_control_plugin`、`src/amr_dispatcher_ros/include/amr_dispatcher_ros/ros2_control_plugin`（先确认无文件）。
- [ ] 脚本统一 `set -euo pipefail`：`build.sh`、`benchmark.sh`、`record_topics.sh`、`run_demo.sh`、`container_build.sh`、`docker/entrypoint.server.sh`。
- [ ] Dockerfile：尝试 `docker manifest inspect ros:jazzy-ros-base` 取 digest 钉死；环境无 docker 则在 FROM 行加注释说明滚动风险。CI checkout 升 v4（C2 内）。

**C 收口：** `bash scripts/check_all.sh` 全绿且生成的 matrix 无虚构条目。

---

## 最终验证（主线程执行）

- [ ] 全量：core ctest + BT ctest + colcon build + check_all 一条龙，贴证据。
- [ ] `/code-review` 或 requesting-code-review 审 diff。
- [ ] 按工作流分 3~4 个 commit（用户已授权修复；提交风格对齐仓库历史）。
