# AMR Dispatcher API Specification

## 1. ROS 2 话题接口 (Topics)

### 发布话题 (Published Topics)
| 话题名称 | 消息类型 | 频率 | 描述 |
|---|---|---|---|
| `/odom` | `nav_msgs/msg/Odometry` | 50 Hz | 底盘里程计反馈（位置、姿态四元数、线速度、角速度） |
| `/chassis/link_health` | `amr_dispatcher_interfaces/msg/ChassisLinkHealth` | 2 Hz | 底盘硬件通信链路健康指标（丢包率、CRC错、RTT时延） |
| `/dispatcher/state` | `amr_dispatcher_interfaces/msg/DispatcherState` | 2 Hz | 调度器全局状态（当前策略、队列深度、活跃任务列表、死锁状态、`vehicle_links[]` 受管车辆状态） |
| `/dispatcher/events` | `amr_dispatcher_interfaces/msg/MissionEvent` | 事件触发 | 任务生命周期状态跃迁审计事件 |
| `/dispatcher/topology_state` | `amr_dispatcher_interfaces/msg/TopologyState` | 2 Hz | 动态路网拓扑状态（任务活跃边、预约锁边及持有者、死锁站点、死锁标志），供 safety_gate 与 RViz 可视化消费 |
| `/cmd_vel_safe` | `geometry_msgs/msg/Twist` | 50 Hz | 经过安全门控仲裁后的最终安全控制速度（下发至底盘驱动） |
| `/safety/state` | `amr_dispatcher_interfaces/msg/SafetyState` | 10 Hz | 安全门控运行状态（是否允许运动、当前限速比例、阻断源） |
| `/tracking/error` | `amr_dispatcher_interfaces/msg/TrackingError` | 20 Hz | 循迹跟踪误差（横向误差、航向误差、前视距离、到达标记） |
| `/system/aggregated_link_health` | `amr_dispatcher_interfaces/msg/ChassisLinkHealth` | 2 Hz | 聚合系统多链路的整体健康评级 |

#### 多车模式（per-vehicle）话题

配置 `vehicle_ids`（逗号分隔的 local_sim 受管车清单）或 `vda5050_vehicles` 后，
下列话题按 **`{vehicle_id}` 后缀**隔离，每台车一份。**兼容红线**：`vehicle_ids`
与 `vda5050_vehicles` 都为空（legacy 单车模式）时，下列话题名**逐字不变**。

| 话题形状 | 消息类型 | 描述 |
|---|---|---|
| `/plan/{vid}` | `nav_msgs/msg/Path` | 该车的待跟踪路点序列（VDA 5050 车**不发布**：其运动由自带控制器执行） |
| `/odom/{vid}` | `nav_msgs/msg/Odometry` | 该车里程计反馈 |
| `/tracking/error/{vid}` | `amr_dispatcher_interfaces/msg/TrackingError` | 该车循迹跟踪误差与到达标记 |
| `/cmd_vel_raw/{vid}` | `geometry_msgs/msg/Twist` | 该车循迹控制器的原始期望速度 |
| `/cmd_vel_safe/{vid}` | `geometry_msgs/msg/Twist` | 该车安全门控后的最终速度 |
| `/safety/heartbeat/path_tracker/{vid}` | `std_msgs/msg/Bool` | 该车循迹节点心跳 |
| `/safety/heartbeat/chassis_driver/{vid}` | `std_msgs/msg/Bool` | 该车底盘驱动心跳 |
| `/chassis/link_health/{vid}` | `amr_dispatcher_interfaces/msg/ChassisLinkHealth` | 该车链路健康指标 |

TF frame 同步隔离：`{vid}/odom`、`{vid}/base_link`（**禁止**多车抢同一 frame）。

`/safety/estop`、`/safety/bumper` 是**全局安全信号** —— 任何位置的急停/碰撞都
必须让全部车辆停车，因此多车模式下每台车的安全门**同时订阅**全局话题与本车话题。

### 订阅话题 (Subscribed Topics)
| 话题名称 | 消息类型 | 描述 |
|---|---|---|
| `/cmd_vel_raw` | `geometry_msgs/msg/Twist` | 循迹控制器或遥控节点发出的原始期望速度 |
| `/safety/estop` | `std_msgs/msg/Bool` | 紧急停止按钮状态输入（true 触发全停） |
| `/safety/bumper` | `std_msgs/msg/Bool` | 碰撞保险杠接触信号输入（true 触发全停） |
| `/safety/heartbeat/chassis_driver` | `std_msgs/msg/Bool` | 底盘驱动心跳（chassis_driver 发布，safety_gate 分源订阅计时） |
| `/safety/heartbeat/path_tracker` | `std_msgs/msg/Bool` | 循迹控制器心跳（path_tracker 发布，safety_gate 分源订阅计时） |
| `/plan` | `nav_msgs/msg/Path` | 全局规划器发布的待跟踪路点序列 |

---

## 2. ROS 2 服务接口 (Services)

### `/dispatcher/submit_order` (`amr_dispatcher_interfaces/srv/SubmitOrder`)
提交新任务至调度队列。
- **请求参数**:
  - `string order_id`: 业务订单号
  - `string mission_id`: 任务唯一标识（留空自动生成）
  - `string mission_type`: 任务类型（`transport` / `charge` / `patrol`）
  - `string pickup_station`: 取货站点标识
  - `string dropoff_station`: 卸货站点标识
  - `int32 priority`: 优先级数值（越大越优先）
  - `int64 deadline_unix_ms`: 截止时间戳毫秒（供 EDF 调度策略）
  - `int32 expected_seconds`: 预估执行秒数（供 SJF 调度策略）
  - `string assigned_vehicle_id`: **显式指定承接车辆** id（空 = 由选车策略决定）。
    显式指定不豁免任何硬约束：车辆必须在线、未隔离、未忙、具备所需能力、
    电量达标、位置有效且状态新鲜，否则该单按三态语义处置（留队/暂停/判负）。
    *该字段为末尾追加，旧客户端不受影响。*
- **响应参数**:
  - `bool accepted`: 是否入队成功
  - `string mission_id`: 最终任务 ID
  - `string message`: 拒绝原因或确认消息

### `/dispatcher/cancel_order` (`amr_dispatcher_interfaces/srv/CancelOrder`)
按任务号或整单取消任务。
- **请求参数**:
  - `string order_id`: 按订单号整单取消
  - `string mission_id`: 按任务唯一 ID 精确取消
- **响应参数**:
  - `bool success`: 是否取消成功
  - `uint32 canceled_count`: 实际从队列与执行态移除的任务数
  - `string message`: 处置描述

### `/dispatcher/pause_mission` (`amr_dispatcher_interfaces/srv/PauseMission`)
暂停指定任务或订单。
- **请求参数**: `string mission_id`, `string reason`
- **响应参数**: `bool success`, `string message`

### `/dispatcher/resume_mission` (`amr_dispatcher_interfaces/srv/ResumeMission`)
恢复被暂停的任务。
- **请求参数**: `string mission_id`
- **响应参数**: `bool success`, `string message`

### `/dispatcher/reserve_resource` (`amr_dispatcher_interfaces/srv/ReserveResource`)
向交通管制中心申请独占拓扑边或节点。
- **请求参数**:
  - `string resource_id`: 资源标识，如 `route_edge:station_A->station_B` 或 `route_node:cross_1`
  - `string holder_id`: 占用者标识（机器人 ID 或任务 ID）
  - `int64 timeout_ms`: 超时自动释放毫秒数
- **响应参数**:
  - `bool success`: 是否独占成功
  - `string token`: 释放时所需的凭据 Token
  - `string message`: 处置描述

### `/dispatcher/release_resource` (`amr_dispatcher_interfaces/srv/ReleaseResource`)
凭 Token 释放已申请的拓扑互斥资源。
- **请求参数**: `string resource_id`, `string holder_id`, `string token`
- **响应参数**: `bool success`, `string message`

### `/dispatcher/force_clear_vda5050_link` (`amr_dispatcher_interfaces/srv/ForceClearVda5050Link`)
操作员强制清除**离线** VDA 5050 链路（Spec §5.2 人工强解半边）。

**安全边界**（服务端复验，前端隐藏按钮不是安全边界）：
- 通信失联 ≠ 可以释放路权。离线只说明通信异常，不证明车辆已停止。
  本操作是**操作员的人工声明**，由操作员承担车辆物理占位的安全责任。
- 强清**不摘车**：车辆转入**隔离态**（`quarantined`），仍可见/可审计，
  但不再参与自动分配；恢复需调用 `/dispatcher/restore_vda5050_link`。
- 在线车辆 → 拒绝（绝不误杀活车）；已隔离车辆 → 拒绝（避免重复操作）。

- **请求参数**:
  - `string vehicle_id`: 目标车辆 id（须在受管清单内）
  - `string confirm_vehicle_id`: 操作员确认的车辆 id，**必须与 `vehicle_id` 完全一致**（防打错车）
  - `string operator_id`: 操作员标识（必填，写入审计）
  - `string reason`: 操作原因（必填，写入审计）
- **响应参数**:
  - `bool success`: 准入校验结果
  - `string message`: 拒绝原因或操作摘要
  - `string[] released_resources`: 实际释放的路权锁 id 清单
  - `bool quarantined`: 受理后车辆已转入隔离态

### `/dispatcher/restore_vda5050_link` (`amr_dispatcher_interfaces/srv/RestoreVda5050Link`)
解除受管车辆的隔离态，使其重新参与自动调度。
- **前置**: 该车曾被 `ForceClearVda5050Link` 隔离。
- **条件**: 车辆在线 + 操作员已核实车辆物理位置（`position_verified`）。
  位置未核实的车可能仍占着通道，放回调度池会撞车。
- **请求参数**:
  - `string vehicle_id`, `string confirm_vehicle_id`（须完全一致）,
    `string operator_id`（必填）, `string reason`（必填）, `bool position_verified`
- **响应参数**: `bool success`, `string message`

### `/v2/*` 核心调度服务（12 个）

与上方 legacy 服务并存，均由 `dispatcher_node` 提供；字段以 `src/amr_dispatcher_interfaces/srv/*.srv` 定义为准。
实现状态：「完整实现」= 字段全部来自真实计算；「基础实现」= 服务可正常调通、字段已生成，但部分输出当前为占位/最小实现，正在完善中。

| 服务名 | srv 类型 | 请求字段 | 响应字段 | 实现状态 |
|---|---|---|---|---|
| `/v2/enqueue_mission` | `EnqueueMission` | `mission_file`, `priority`, `start_if_idle` | `success`, `message`, `queue_size` | 完整实现 |
| `/v2/cancel_queued_mission` | `CancelQueuedMission` | `mission_id`, `cancel_active` | `success`, `message`, `queue_size` | 完整实现 |
| `/v2/preempt_mission` | `PreemptMission` | `mission_file`, `priority` | `success`, `message`, `mission_id`, `queue_size` | 完整实现 |
| `/v2/reprioritize_queued_mission` | `ReprioritizeQueuedMission` | `mission_id`, `priority` | `success`, `message`, `queue_size` | 完整实现 |
| `/v2/estimate_mission_cost` | `EstimateMissionCost` | `mission_file`, `nominal_speed_mps`, `battery_voltage` | `success`, `message`, `mission_id`, `waypoint_count`, `distance_m`, `eta_sec`, `battery_drop_v`, `projected_battery_voltage`, `battery_sufficient` | 基础实现 |
| `/v2/block_station_route` | `BlockStationRoute` | `from_station`, `to_station`, `reason` | `success`, `message`, `resource_id` | 完整实现 |
| `/v2/detect_traffic_deadlock` | `DetectTrafficDeadlock` | （无请求字段） | `success`, `message`, `deadlocked`, `mission_ids[]`, `conflict_resource_ids[]`, `conflict_station_ids[]`, `descriptions[]` | 完整实现 |
| `/v2/list_stations` | `ListStations` | （无请求字段） | `success`, `message`, `station_ids[]`, `frame_ids[]`, `x[]`, `y[]`, `yaw[]` | 基础实现 |
| `/v2/list_traffic_reservations` | `ListTrafficReservations` | （无请求字段） | `success`, `message`, `resource_ids[]`, `owner_ids[]`, `reservation_types[]` | 完整实现 |
| `/v2/reserve_facility_resource` | `ReserveFacilityResource` | `resource_id`, `holder_id`, `mission_id`, `release_existing_for_holder` | `success`, `message`, `resource_id`, `holder_id`, `status`（`RESERVED`/`OCCUPIED`） | 完整实现 |
| `/v2/get_operator_snapshot` | `GetOperatorSnapshot` | `event_limit` | `success`, `message`, `mission_state`, `active_mission_id`, `mission_active`, `paused`, `queue_size`, `queued_*`, `paused_workflow_order_ids[]`, `fleet_*`, `remote_task_*`, `facility_*`, `business_*`, `recent_event_*`（完整字段见 `.srv`） | 基础实现 |
| `/v2/validate_site_config` | `ValidateSiteConfig` | `bundle_dir`, `version_id` | `success`, `message`, `version_id`, `checked_files[]`, `errors[]`, `diff_report` | 基础实现 |

---

## 3. ROS 2 Action 接口 (Actions)

### `/navigate_sequence` (`amr_dispatcher_interfaces/action/NavigateSequence`)
按顺序巡航途经多个目标站点。
- **Goal**:
  - `geometry_msgs/PoseStamped[] waypoints`: 巡航路点目标序列
  - `string[] station_names`: 途经站点的业务名称
  - `bool loop`: 是否循环巡航
- **Feedback**:
  - `int32 current_index`: 当前正前往的目标索引
  - `int32 total_waypoints`: 总路点数
  - `string current_station`: 当前目标站点名
  - `string state`: 状态（`NAVIGATING` / `AT_STATION` / `FAILED`）
  - `float64 distance_to_current_goal_m`: 距当前目标剩余欧式距离
- **Result**:
  - `bool success`: 完整航线是否顺利完成
  - `string message`: 最终报告
  - `int32 completed_waypoints`: 成功到达的路点数
  - `int32 failed_waypoints`: 失败的路点数

### `/execute_mission` (`amr_dispatcher_interfaces/action/ExecuteMission`)
长时任务派发的主入口：排队 → 抢锁 → 执行全生命周期流式反馈，支持客户端取消与优先级抢占。
- **Goal**:
  - `string mission_id`: 任务唯一标识（留空、或与在途任务重复，goal 将被 REJECT）
  - `uint8 priority`: 优先级 1–255（数值越大越优先，具备抢占权）
  - `string target_station`: 目标站点标识（如 `station_dropoff_1`）
  - `float64 deadline_seconds`: 截止时间约束（供 EDF 调度策略计算）
  - `string[] required_resources`: 预申请的拓扑边或设施锁（须全部原子预约成功，任一失败即 REJECT，不残留部分锁）
- **Feedback**:
  - `uint8 state_code`: 0 `QUEUED` / 1 `WAITING_LOCK` / 2 `EXECUTING` / 3 `YIELDING` / 4 `RECOVERING`
  - `string current_node`: 当前经过的拓扑节点
  - `float32 percent_complete`: 完成百分比 0.0–100.0
  - `float64 estimated_remaining_sec`: 预估剩余秒数
- **Result**:
  - `bool success`: 任务是否顺利完成
  - `int32 error_code`: 0 `OK` / 1 `RESOURCE_DEADLOCK` / 2 `TIMEOUT` / 3 `PREEMPTED` / 4 `HARDWARE_ERROR`
  - `string message`: 终态描述
  - `float64 total_duration_sec`: 端到端总耗时

---

## 4. 调度器参数 (dispatcher_node)

多车车队化的关键参数（`config/dispatcher.yaml`，`dispatcher_node.ros__parameters`）。
**兼容红线**：`vehicle_ids` 与 `vda5050_vehicles` 都为空时（legacy 单车模式），
话题名、选车流程、派发节奏与改动前逐字一致。

| 参数 | 默认 | 说明 |
|---|---|---|
| `vehicle_ids` | `""` | 本调度器管理的 **local_sim** 受管车清单（逗号分隔 vehicle_id）。非空时为每台车建立 `/plan/{vid}`、`/odom/{vid}`、`/tracking/error/{vid}` 的隔离话题与独立执行槽位。 |
| `vda5050_vehicles` | `""` | 受管 **VDA 5050** 外部车清单（逗号分隔 serialNumber）。这些车**不发布**本地 `/plan`、**不占用**本地执行槽位：运动由其自带控制器执行，完成判定为 state 报文 `lastNodeId` 推进到订单终点。 |
| `fleet_catalog_file` | `""` | 车辆静态目录（`config/fleet.yaml`）。`robots[].backend` 取 `local_sim`（默认）/ `vda5050`；空 = 用上述两份清单 + 默认静态字段。 |
| `required_capability` | `"transport"` | 选车所需能力标签（空 = 不限制）。 |
| `min_battery_percent` | `0.0` | 选车电量下限，低于该值的车不合格。 |
| `vehicle_state_max_age_ms` | `5000` | 车辆状态新鲜度上限；超时视为状态不确定（暂停自动分配，等状态恢复），**不**盲派。 |
| `assignment_scan_depth` | `8` | 一次 tick 最多考察前 N 个排队任务（队首阻塞规避：队首不可派时继续找后面可派的）。 |
| `traffic_mode` | `"static"` | 交通协调模式：`static` = 静态排他锁（现网行为）；`time_space` = 时空窗预约（多车可错峰并行）。 |
| `time_space_cruise_speed_mps` | `1.0` | 时空窗模式把几何长度换算成时间占用区间用的巡航速度。 |
| `time_space_accel_overhead_sec` | `1.0` | 每次进入资源额外支付的起步/刹停时间。 |
| `time_space_turn_overhead_sec` | `0.5` | 按路径拐点数计的转弯附加时间。 |
| `time_space_safety_margin_m` | `0.2` | 车辆尺寸安全余量（沿路径方向额外占用长度）。 |
| `time_space_min_headway_sec` | `2.0` | 最小安全间隔：后车进入同一资源前必须与前车拉开该间隔。 |

### 选车结果三态语义

`PlanVehicleAssignment`（纯函数，核心库）返回四类结果，ROS 侧据此处置：

| 结果 | 触发条件 | 调度处置 |
|---|---|---|
| `kAssigned` | 全部硬约束满足 | 派发 |
| `kDeferred` | **暂时**不可用（车忙 / 离线 / 隔离 / 状态陈旧 / 无有效位置） | **任务留队等待**，下个 tick 重试（不判负） |
| `kIndeterminate` | 状态**不确定**（从未收到该车状态报文） | **暂停自动分配**，等状态恢复 |
| `kRejected` | **永久**不满足（显式指定车不存在 / 禁用 / 能力不符 / 电量不足） | 该单判负，附明确原因 |

硬约束（`kAssigned` 必须全部满足，**显式指定车辆同样适用**）：
`online ∧ enabled ∧ !busy ∧ !quarantined ∧ 具备能力 ∧ battery ≥ 阈值 ∧ position_valid ∧ 状态新鲜`。

### 时空窗交通协调（`traffic_mode: time_space`）

- **逐资源区间**：一条路径涉及多个资源（边=通道、节点=路口），每个资源单独
  计算进入/离开时刻；路口窗口 = 进入路口到驶出路口的完整区间。
- **全有或全无**：整条路径的预约必须全部成功才提交；任一资源失败则全部回滚，
  不留半套预约。
- **窗口到期 ≠ 资源已释放**：物理资源仍需车辆**确认驶离**（`lastNodeId` 推进，
  Spec §5.3 逐段解锁）才可让后车进入；执行偏离靠 `ExtendTimeSpace` 延期扩窗。
- **默认仍为 `static`** —— 时空窗模式在多车并发与单测全绿后才显式启用。

---

## 5. REST HTTP 网关接口 (REST API)

默认监听端口：`8080`

### `GET /api/v1/state`
查询调度系统当前整体健康度与队列信息。
- **Response**:
```json
{
  "comparator": "priority_fifo",
  "queue_size": 3,
  "queue_capacity": 128,
  "active_count": 1,
  "deadlock": false
}
```

### `POST /api/v1/orders`
提交订单，支持 `idempotency_key` 幂等性防护。
- **Query Parameters**:
  - `id`: 任务 ID
  - `from`: 起始站点
  - `to`: 目标站点
  - `prio`: 优先级（整数）
  - `idempotency_key`: 幂等性令牌（相同 key 重复请求将被自动去重过滤）
  - `vehicle`: **显式指定承接车辆** id（空 = 由选车策略决定）。
    显式指定不豁免硬约束：车辆必须在线、未隔离、未忙、能力匹配、电量达标、
    位置有效且状态新鲜；否则该单按三态语义处置（留队等待 / 暂停自动分配 / 判负）。
- **Response (Success)**:
```json
{
  "status": "ok",
  "accepted": true,
  "mission_id": "task_1001"
}
```

### `GET /api/v1/vehicle_links`
查询**受管车辆**的链路与调度状态（结构化消息 `VehicleLinkState`，
不依赖平行数组）。覆盖 local_sim 与 vda5050 两类后端。
- **Response**:
```json
{
  "vehicle_links": [
    {
      "vehicle_id": "amr_01",
      "backend": "local_sim",
      "online": true,
      "busy": false,
      "quarantined": false,
      "position_valid": true,
      "current_station_id": "station_pickup_A",
      "battery_percent": 100.0,
      "state_age_ms": 42,
      "last_error": ""
    }
  ]
}
```

### `POST /api/v1/force_clear_link`
操作员强制清除**离线**链路（危险操作，请求体为 JSON）。
- **Request Body**:
```json
{
  "vehicle_id": "mock_amr_01",
  "confirm_vehicle_id": "mock_amr_01",
  "operator_id": "operator_zhang",
  "reason": "车辆报废，现场已确认无人"
}
```
四个字段**全部必填**；`confirm_vehicle_id` 必须与 `vehicle_id` 完全一致。
服务端复验准入条件（在线车拒 / 已隔离拒 / id 不匹配拒）——
前端隐藏按钮不是安全边界。
- **Response (Success)**:
```json
{
  "status": "ok",
  "message": "Force-cleared offline link for mock_amr_01, released 3 resource(s) | ...",
  "quarantined": true,
  "released_resources": ["route_node:station_pickup_A", "route_edge:station_dropoff_B__station_pickup_A"]
}
```
- **失败**: 准入校验不通过 → `409 Conflict` `{"status":"rejected","message":"..."}`;
  缺字段 → `400` `{"error":"missing_fields","detail":"..."}`;
  服务不可用 → `503`；调用超时 → `504`。

### `POST /api/v1/restore_link`
解除车辆隔离态，使其重新参与自动调度（请求体为 JSON）。
- **Request Body**:
```json
{
  "vehicle_id": "mock_amr_01",
  "confirm_vehicle_id": "mock_amr_01",
  "operator_id": "operator_zhang",
  "reason": "车辆已恢复在线，位置已现场核实",
  "position_verified": true
}
```
`position_verified` 表示操作员已核实车辆物理位置与通道占用 ——
位置未核实的车可能仍占着通道，放回调度池会撞车。
- **Response (Success)**:
```json
{ "status": "ok", "message": "Restored vehicle mock_amr_01 to dispatch pool | ..." }
```
- **失败**: 准入校验不通过 → `409 Conflict`；缺字段 → `400`；服务不可用 → `503`。

### `GET /api/operator/snapshot`
查询完整运营快照大盘（与 `GET /api/v1/state` 为同一实现、同一响应体，路由别名）。
- **Response**:
```json
{
  "comparator": "priority_fifo",
  "strategy": "priority_fifo",
  "queue_size": 3,
  "queue_capacity": 128,
  "active_count": 1,
  "active_mission_id": "m_1",
  "deadlock": false,
  "deadlock_detected": false,
  "estop_active": false,
  "safety_severity": "INFO",
  "chassis_backend": "Serial",
  "chassis_healthy": true,
  "chassis_loss_rate": 0.0,
  "recent_events": [
    { "mission_id": "m_1", "event": "ACCEPTED", "reason": "" }
  ]
}
```

### `POST /api/operator/estop`
一键急停：向 `/safety/estop` 话题发布 `true`，全系统安全链随即熔断。
- **Response**: `{"status": "ok", "estop_engaged": true}`
- *注：发布即返回，不含下游执行确认；解除急停请走 ROS 2 话题或安全门接口。*

### `GET /api/system/ready`
系统就绪探针（代理 `/system/ready` 服务），供上层业务系统与负载均衡做健康检查。
- **Response (Ready)**: `{"ready": true, "healthy": true, "state": "READY", "message": "..."}`
  （`state` 取值：`INITIALIZING` / `READY` / `DEGRADED` / `FAULT`；网关仅回写 ready/healthy/state/message 四个字段）
- **Not Ready** → `503 Service Unavailable`，如 `{"ready": false, "state": "INITIALIZING", "message": "Lifecycle Coordinator not ready"}`
- **Probe Timeout** → `504 Gateway Timeout`，`{"error": "readiness_probe_timeout"}`

### `POST /api/workflows/cancel`
按任务 ID 取消排队任务（别名 `POST /api/v1/cancel`；后端调用 `/dispatcher/cancel_order`）。
- **Query Parameters**: `id` — 任务/订单 ID
- **Response**: `{"status": "ok", "canceled": true}`（`canceled` 反映 CancelOrder 的 `success`）
- **错误**: 下游服务不可用 → `503` `{"error": "service_unavailable"}`；调用超时 → `504` `{"error": "timeout"}`

---

## 6. Web 操作台浏览器端验收

`tools/operator_console.html` 是零依赖单文件控制台。上述 REST 端点的**浏览器端**
行为由 `./scripts/check_web_console_e2e.sh` 自动化验收（headless Chrome + CDP 真实点击）：

| 阶段 | 断言 |
|---|---|
| panel | 受管车辆面板渲染（id / backend / 在线态 / 位置 / 电量 / 状态时延） |
| force_clear_online | 在线车强清被**服务端**拒绝（409）——前端隐藏按钮不是安全边界 |
| force_clear_offline | 缺 `operator_id` / `reason` 时**根本不发请求**；离线车强清受理 → 车辆转入**隔离态**（面板可见） |
| restore_guard | 未勾选"已核实车辆物理位置"时**根本不发请求** |
| restore | 勾选后显式恢复 → 解除隔离，车辆重回调度池 |

环境隔离：ROS 栈在容器内跑独立域（`ROS_DOMAIN_ID=82` / MQTT :18890 / REST :18092），
浏览器在宿主机通过容器 IP 直连网关。
