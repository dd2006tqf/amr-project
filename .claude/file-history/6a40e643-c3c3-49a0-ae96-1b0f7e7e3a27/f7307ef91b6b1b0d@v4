# AMR Dispatcher API Specification

## 1. ROS 2 话题接口 (Topics)

### 发布话题 (Published Topics)
| 话题名称 | 消息类型 | 频率 | 描述 |
|---|---|---|---|
| `/odom` | `nav_msgs/msg/Odometry` | 50 Hz | 底盘里程计反馈（位置、姿态四元数、线速度、角速度） |
| `/chassis/link_health` | `amr_dispatcher_interfaces/msg/ChassisLinkHealth` | 2 Hz | 底盘硬件通信链路健康指标（丢包率、CRC错、RTT时延） |
| `/dispatcher/state` | `amr_dispatcher_interfaces/msg/DispatcherState` | 2 Hz | 调度器全局状态（当前策略、队列深度、活跃任务列表、死锁状态、`vehicle_links[]` 受管车辆状态；相位 11 追加 `overlay_active` / `overlay_revision` / `blocked_edges` / `one_way_edges` 供控制台免服务调用展示 —— rosidl 末尾追加，旧订阅方不受影响） |
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

### `/v2/*` 核心调度服务（15 个）

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
| `/v2/block_edge` | `BlockEdge` | `from_station`, `to_station`, `action`, `direction`, `reason`, `operator_id`, `expected_map_hash`, `confirm`, `allow_edge_under_vehicle` | `success`, `error_code`, `message`, `resource_id`, `impacted_vehicles[]`, `impacted_missions[]`, `rerouted`, `overlay_revision` | 完整实现 |
| `/v2/topology_hot_update` | `TopologyHotUpdate` | `overlay_yaml`, `expected_map_hash`, `operator_id`, `reason`, `dry_run`, `clear`, `allow_edge_under_vehicle`, `confirm` | `success`, `error_code`, `message`, `unreachable_nodes[]`, `impacted_vehicles[]`, `impacted_missions[]`, `reroute_failures[]`, `catalog_map_hash`, `overlay_revision`, `file_written` | 完整实现 |
| `/v2/get_topology_overlay` | `GetTopologyOverlay` | `include_yaml` | `success`, `message`, `overlay_yaml`, `revision`, `blocked_edge_count`, `one_way_edge_count`, `marker_count`, `base_map_hash`, `effective_map_hash`, `overlay_active` | 完整实现 |

> 后三个是相位 11 新增的"目录感知"拓扑热生效接口，见 §5.5。
> 旧的 `/v2/block_station_route` **逐字节保持不变**（仅预约表，不改规划），两者并存。

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
| `traffic_mode` | `"static"` | 交通协调模式：`static` = 静态排他锁（现网行为）；`time_space` = 时空窗预约（多车可错峰并行）；`cbs` = 高层多智能体协同寻路（CBS 冲突树搜索 + 自动降级至 Prioritized Planning）。未知值回落 `static`。 |
| `facility_catalog_file` | `""` | 现场物理设施目录路径（`config/facilities.yaml`）。空 = 不启用物理设施控制。支持 Modbus-TCP / 现场 PLC 协议接入与自动卷帘门/跨楼层电梯联锁。 |
| `auto_charging_enabled` | `false` | 是否开启低电量自动机动插充电单（批次 6.2）。默认 false。开启时必须同时指定 `auto_charging_station`。 |
| `auto_charging_station` | `""` | 充电站站点 ID（需存在于 `stations.yaml`）。若开启且未配站则节点报错并自动回退关闭。 |
| `auto_charging_opportunity_threshold_percent` | `30.0` | 机会充电阈值：车辆空闲且电量低于该值时插入低优先级充电单。 |
| `auto_charging_critical_threshold_percent` | `15.0` | 紧急充电阈值：低于该值插入高优先级抢占式充电单。 |
| `auto_charging_opportunity_priority` | `0` | 机会充电单优先级（默认 0 = 普通 FIFO）。 |
| `auto_charging_critical_priority` | `5` | 紧急充电单优先级（默认 5 = 优先出队）。 |
| `time_space_cruise_speed_mps` | `1.0` | 时空窗模式把几何长度换算成时间占用区间用的巡航速度。 |
| `time_space_accel_overhead_sec` | `1.0` | 每次进入资源额外支付的起步/刹停时间。 |
| `time_space_turn_overhead_sec` | `0.5` | 按路径拐点数计的转弯附加时间。 |
| `time_space_safety_margin_m` | `0.2` | 车辆尺寸安全余量（沿路径方向额外占用长度）。 |
| `time_space_min_headway_sec` | `2.0` | 最小安全间隔：后车进入同一资源前必须与前车拉开该间隔。 |

#### 相位 11 新增：运行时拓扑叠加层

| 参数 | 默认 | 说明 |
|---|---|---|
| `topology_overlay_file` | `""` | 运行时拓扑叠加层的持久化路径。**空 = 功能关闭**（不读、不写、`/v2/*` 拓扑服务返回 `catalog_unavailable` 语义的拒绝），现网行为逐字不变。生产由 launch 注入**绝对路径**——`HandleMapUpload` 写的是相对 `config/generated/stations.yaml`，沿用相对路径会踩同一个 cwd 陷阱。 |
| `topology_allow_edge_under_vehicle` | `false` | 允许"车可能正压在被封边上"时仍提交（该边的锁留给在途车，等它自己释放）。默认 `false` = 严格拒绝。**单向反转不受此参数影响**，恒 409。 |
| `safety_stale_sec`（协调器） | `2.0` | 协调器判"安全状态新鲜"的阈值。安全门停发后 `motion_allowed` 会永远停在最后一次的值上，只看值不看时间等于安全门死了以后就绪仍是 `true`。 |
| `dispatcher_stale_sec`（协调器） | `3.0` | 协调器判"dispatcher 可用"的阈值。**这是"容器 Up 8 天、dispatcher 已死 8 天、网关照样 200"的直接病根**：旧实现里 `get_state` 服务不可用时直接 return，状态永远停在最后一次读到的 `active`。修复后不可用会显式标记，`/system/ready` 报 `INITIALIZING` + `DispatcherNode unavailable: ...`，`/system/healthy` 报 `false`。 |

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

### 时空窗与高层协同寻路（`traffic_mode: time_space` / `cbs`）

- **逐资源区间**：一条路径涉及多个资源（边=通道、节点=路口），每个资源单独
  计算进入/离开时刻；路口窗口 = 进入路口到驶出路口的完整区间。
- **全有或全无**：整条路径的预约必须全部成功才提交；任一资源失败则全部回滚，
  不留半套预约。
- **窗口到期 ≠ 资源已释放**：物理资源仍需车辆**确认驶离**（`lastNodeId` 推进，
  Spec §5.3 逐段解锁）才可让后车进入；执行偏离靠 `ExtendTimeSpace` 延期扩窗。
- **CBS / MAPF 协同（`traffic_mode: cbs`）**：
  - 低层时空 $A^*$ 引入站点原地等待（`Wait In Place`）动作；
  - 高层约束树（Constraint Tree）检测并分支消解顶点重叠与通道对撞；
  - 终点驻留保护（Goal-Hold）：车辆抵达终点后持续占据站点，杜绝后续车穿身冲撞；
  - 自适应降级门面（`SolveMAPF`）：任务规模 $> 6$ 或 CBS 搜索超预算时，平滑降级至优先级时空规划（Prioritized Planning / PBS），保证毫秒级无冲突收敛；
  - 窗口转换统一注入 `MotionTimingConfig`（起步刹停/转弯/尺寸余量/最小安全间隔 `min_headway_sec`），与时空窗模式口径严格一致。
- **默认仍为 `static`** —— 高级模式在多车并发与集成验收全绿后按需显式启用。

### 调度状态持久化与审计落盘（`state_snapshot_file` / `audit_log_file`）

| 参数 | 默认 | 说明 |
|---|---|---|
| `state_snapshot_file` | `""` | 调度状态快照路径。**空 = 关闭持久化**（进程重启即失，现网行为不变）。 |
| `state_snapshot_debounce_sec` | `2.0` | 变更合并写盘窗口：派发/完成/取消/暂停/快照脏标记合并后再落盘，避免高频磁盘写。 |
| `audit_log_file` | `""` | 审计 JSONL 路径。空 = 不落审计（仅保留 `RCLCPP_WARN`）。 |

**快照内容**：`format_version` / `map_hash` / `next_sequence` / 排队任务（含
`priority`、`sequence`、`deadline_unix_ms`、`expected_seconds`、`assigned_vehicle_id`）/
暂停订单 id 列表 / 在途任务 / 隔离车辆 id 列表。

**恢复语义**：

| 情形 | 处置 |
|---|---|
| 文件不存在 | 视为首次启动（`No dispatch state snapshot ... (fresh start)`） |
| 文件损坏 / 版本不认识 / 未知状态枚举 | **拒绝恢复**并保留现场文件，从空队列启动 + `ERROR` 日志 |
| `map_hash` 与当前拓扑不一致 | **拒绝恢复**（不拿旧拓扑的队列去派发），从空队列启动 |
| 快照里"在途"的任务 | 重启后**重新入队**——新进程没有任何车在跑它们，不假装在途 |
| 暂停订单下的任务 | 先不入队，待暂停标记生效后再补回（避免被容量上限挤掉） |
| `next_sequence` | 取快照游标，且不低于已见最大 `sequence`（杜绝恢复后撞号） |
| 隔离车辆 | 恢复为隔离态（仍在受管清单，但不参与自动分配） |

**落盘原子性**：`tmp` → `flush` 校验 → 回读反序列化校验 → `rename(2)`；
任一步失败删除 tmp、保留旧文件，脏标记不清（下个周期重试）。

**审计记录**（JSONL，一行一条）：`stamp` / `operator_id` / `action` /
`vehicle_id` / `mission_id` / `reason` / `released_resources[]`。
当前会写审计的操作：`force_clear`、`restore`、`restore_state`（启动恢复）、
`restore_state_rejected`（`map_hash` 不匹配拒绝恢复）。

验收：`./scripts/check_state_persistence_e2e.sh`（独立域 87 / REST 18098 /
MQTT 18891）——3 单（含 1 单暂停）+ 隔离车辆 → `kill -9` → 重启 →
队列/暂停标记/隔离态保留、`next_sequence` 不回退、审计落盘、`map_hash`
不匹配时拒绝恢复。

---

## 5. REST HTTP 网关接口 (REST API)

默认监听端口：`8080`

### 5.1 网关参数与安全边界

| 参数 | 默认 | 说明 |
|---|---|---|
| `port` | `8080` | 监听端口。 |
| `html_path` | `tools/operator_console.html` | 操作台单文件路径（含若干回退路径）。 |
| `auth_token` | `""` | 静态 Bearer Token。**空 = 鉴权关闭**（现网与全部 e2e 行为不变）。非空时除 `GET /`、`GET /index.html` 外全部端点要求 `Authorization: Bearer <token>`，否则 `401`。 |
| `estop_token` | `""` | `/api/operator/estop` 单独口令；空则回退 `auth_token`。 |
| `metrics_token` | `""` | `/metrics` 抓取口令；空则回退 `auth_token`（两者都空 = 不鉴权）。与业务口令分开配，是为了让抓取端只拿到只读指标的读取权。 |
| `idempotency_store_file` | `""` | 幂等键落盘路径；空 = 仅内存（重启即失）。 |
| `idempotency_capacity` / `idempotency_ttl_sec` | `4096` / `3600` | 幂等键容量上限与 TTL。 |
| `cors_allow_origin` | `""` | 允许的跨域来源。**空 = 不下发任何 CORS 头**（操作台与网关同源时不需要）；旧实现无条件下发 `Access-Control-Allow-Origin: *`。 |
| `worker_threads` | `4` | 连接处理线程数。主线程只 `accept`，静默/慢连接只占住一个工作线程。 |
| `max_pending_connections` | `32` | 待处理连接上限；超限立即回 `503 gateway_overloaded` 并关闭，避免无界积压。 |
| `readyz_dispatcher_state_stale_sec` | `3.0` | `/readyz` 判 `dispatcher` 组件新鲜的阈值（`/dispatcher/state` 2Hz）。 |
| `readyz_safety_stale_sec` | `2.0` | 同上，`/safety/state`（空闲 10Hz / 有指令时最高 50Hz）。 |
| `readyz_chassis_stale_sec` | `5.0` | 同上，`/chassis/link_health`（5Hz）。 |
| `readyz_coordinator_stale_sec` | `3.0` | 同上，`/system/ready_flag`（2Hz）。 |
| `readyz_require_chassis` | `true` | 是否把 `chassis` 计入就绪判据。**多车模式必须置 `false`**：那时 `/chassis/link_health` 是 per-vehicle 话题，网关订阅不到全局话题，否则 `/readyz` 永久 503。置 `false` 时该组件在响应与指标里标 `monitored: false`，**不**假装在监控。 |
| `alert_webhook_url` | `""` | 主动告警的 webhook 地址。**空 = 功能关闭**：不起巡检定时器、不起发送线程、进程/线程/话题与关闭前逐字节一致。只支持 `http://`（`https://` 启动时 `ERROR` 并保持禁用——本构建不含 TLS，需要加密请在前面挂本地 http 中继）。 |
| `alert_webhook_timeout_ms` | `3000` | 单次 POST 超时。 |
| `alert_min_interval_sec` | `300` | `firing` 期间的重复提醒间隔；`0` = 只提醒一次（恢复通知不受此限制）。 |
| `alert_queue_capacity` | `64` | 发送队列上限，满时丢**最旧**（过时的 "not ready" 不如刚发生的新故障有用）。 |
| `alert_probe_period_ms` | `1000` | 告警判据的采样周期。 |
| `alert_not_ready_threshold` | `3` | 连续多少次不就绪才升级为告警（滤掉单拍抖动）。 |
| `alert_safety_critical_sec` | `30` | 安全门 `CRITICAL` 需持续多久才告警。 |
| `topology_op_timeout_ms` | `10000` | 拓扑运维端点的下游服务超时。N 车重规划耗时随在途任务数线性增长，故远大于普通业务端点。 |

### 5.1.1 探针：`GET /healthz` 与 `GET /readyz`

两个探针**免鉴权**（`IsAuthExemptPath`），因为容器 `HEALTHCHECK` 无法携带 Bearer Token。
两者都只暴露布尔量与秒数、不含业务数据。

| 端点 | 语义 | 响应 |
|---|---|---|
| `GET /healthz` | **liveness**：网关进程还能处理 HTTP 吗。不调用任何 ROS 服务、不读缓存状态。 | 恒定 `200 {"status":"ok","uptime_seconds":N}` |
| `GET /readyz` | **readiness**：各组件是否新鲜且语义健康。 | `200`（全就绪）或 `503`（任一不就绪） |

`/readyz` 响应结构：

```json
{
  "ready": false,
  "state": "INITIALIZING",
  "message": "waiting for: dispatcher,safety,chassis,coordinator",
  "components": [
    {"component": "dispatcher", "monitored": true, "ok": false, "seen": true,
     "age_seconds": 4.2, "stale_after_seconds": 3.0,
     "reason": "stale", "detail": "state_age_ms=4200"},
    {"component": "safety", "monitored": true, "ok": true, "seen": true,
     "age_seconds": 0.1, "stale_after_seconds": 2.0,
     "reason": "ok", "detail": ""},
    {"component": "chassis", "monitored": false, "ok": true, "seen": false,
     "age_seconds": -1.0, "stale_after_seconds": 5.0,
     "reason": "not_monitored", "detail": "multi_vehicle_mode"},
    {"component": "coordinator", "monitored": true, "ok": false, "seen": false,
     "age_seconds": -1.0, "stale_after_seconds": 3.0,
     "reason": "never_seen", "detail": "no_flag"}
  ]
}
```

字段名以 `health_probe.hpp` 的 `RenderReadinessJson` 为准（这是给编排系统的契约，
不要凭印象改）：数组键是 **`components`**，每项的组件名键是 **`component`**
（不是 `name`）；`reason` 取值 `ok|never_seen|stale|unhealthy|not_monitored`。

判据：`ok = seen && age <= stale_after_seconds && semantic_ok`。语义健康分别是
`dispatcher` = 在发状态、`safety` = `motion_allowed` 且 severity ≠ `CRITICAL`、
`chassis` = `is_healthy`、`coordinator` = `ready_flag == true`。
`!seen` **判为不就绪**（fail closed）——从未收到的组件不能算健康。
`state` 取值：全新鲜且 ok → `READY`；任一组件的 `reason` 是 `never_seen` →
`INITIALIZING`；否则 `DEGRADED`。`monitored=false` 的组件（多车模式下的 `chassis`）
`reason` 恒为 `not_monitored`，**不**计入失败。

**探针快车道**：`/healthz` 与 `/readyz` 由 `accept` 线程读头后**同步**应答，
不进 `pending_` 工作队列。否则 DDS 卡死时 4 个 worker 全被 `wait_for_service` 占住，
探针永远得不到应答——而那正是我们要检测的失效模式本身。

> **HEALTHCHECK 的真实边界**：容器 `healthcheck` 只是可见性与 `docker compose up --wait`
> 的判定依据，**不触发自动重启**。自愈靠 launch 的节点级 `respawn`。禁止接 autoheal：
> 因 `503` 重启会拆掉"健康但安全锁存"的整套栈并丢失调度状态。

**健壮性契约**（`scripts/check_rest_gateway_hardening.sh` 逐条断言）：

| 情形 | 响应 |
|---|---|
| `Content-Length` 非数字 / 负数 / 溢出 | `400 bad_request`，**进程存活** |
| `Content-Length` 超过 1 MiB | `413 payload_too_large` |
| 请求头段超过 8 KiB | `400 bad_request` |
| `Transfer-Encoding: chunked` | `400 bad_request`（网关不实现分块传输，不猜长度） |
| `prio` 非整数 / 越界 | `400 invalid_priority` |
| 路径不是精确匹配（如 `/api/v1/ordersXYZ`） | `404 not_found` |
| 建连后不发数据 | 该连接 5s 收发超时；**其它连接不受影响** |
| 处理过程中抛任何异常 | `400 bad_request`（不再 `std::terminate`） |

> 幂等语义：`idempotency_key` 先占位、**后端确认受理后**才记账。后端不可用
> （`503`）或超时（`504`）时该 key 被回滚，客户端可用同一 key 重试；已受理
> 的 key 重放返回 `{"status":"ignored","reason":"idempotent_duplicate"}`。

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
提交订单，支持 `idempotency_key` 幂等性防护与上游调度契约字段。
- **Query Parameters**:
  - `id`: 任务 ID
  - `order_id`: 业务订单号（若未提供则默认同 id）
  - `from`: 起始站点
  - `to`: 目标站点
  - `prio`: 优先级（整数）
  - `type`: 任务类型（如 `transport`、`charge` 等）
  - `deadline_unix_ms`: 期望截止时间戳（毫秒）
  - `expected_seconds`: 预估执行耗时（秒）
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

### `GET /metrics`
Prometheus 文本格式（`Content-Type: text/plain; version=0.0.4; charset=utf-8`），
供抓取端直接拉取。渲染逻辑在 `amr_dispatcher_tools/metrics_format.hpp`，
格式契约由 `MetricsFormatTest`（宿主 ctest）与 `scripts/check_metrics_e2e.sh`
（真实进程抓取）双重锁定。

| 指标 | 类型 | 来源 |
|---|---|---|
| `amr_queue_size` / `amr_queue_capacity` | gauge | `/dispatcher/state` |
| `amr_active_missions` | gauge | 同上 |
| `amr_deadlock_active` | gauge | 同上 |
| `amr_dispatch_total` | counter | 累计派发成功计数 |
| `amr_dispatch_rejected_total` | counter | 累计派发拒绝计数 |
| `amr_dispatch_deferred_total` | counter | 累计派发推迟计数 |
| `amr_lock_contention_total` | counter | 累计锁冲突拒绝计数 |
| `amr_event_total` | counter | 累计任务生命周期事件计数 |
| `amr_vehicle_online{vehicle_id}` / `amr_vehicle_busy{vehicle_id}` | gauge | `VehicleLinkState[]` |
| `amr_vehicle_quarantined{vehicle_id}` | gauge | 同上 |
| `amr_vehicle_battery_percent{vehicle_id}` | gauge | 同上 |
| `amr_gateway_requests_total{code}` | counter | 网关自身按状态码计数 |
| `amr_ready` | gauge | 0/1，与 `/readyz` **同判据**（同一份缓存快照） |
| `amr_ready_component{component}` | gauge | 0/1，最多 4 条（`dispatcher` / `safety` / `chassis` / `coordinator`） |
| `amr_dispatcher_state_age_seconds` | gauge | 距最近一次 `/dispatcher/state` 的秒数，从未收到 = `-1` |
| `amr_safety_severity_level` | gauge | `0`=INFO `1`=WARN `2`=CRITICAL，`-1`=从未收到 |
| `amr_safety_critical` | gauge | 0/1，severity 为 CRITICAL 时置 1 |
| `amr_topology_overlay_active` | gauge | 0/1，是否存在生效的运行时拓扑叠加层 |
| `amr_topology_blocked_edges` / `amr_topology_one_way_edges` | gauge | 叠加层里的封边 / 单向计数 |
| `amr_topology_overlay_revision` | gauge | 叠加层 revision（无叠加层 = 0） |
| `amr_alert_sent_total` | counter | webhook 发送成功数 |
| `amr_alert_webhook_failures_total` | counter | webhook 发送失败数（**告警通道本身可被抓取侧告警**） |
| `amr_alert_dropped_total` | counter | 队列满时丢弃的告警数 |

**标签基数有界**：拓扑相关指标**绝不用边名打标签**（只给总量），
`amr_vehicle_*` 的标签集来自受管车辆清单（有限）。`/readyz` 与 `/metrics`
读同一份告警定时器维护的缓存快照，保证控制台、指标、探针三处不会互相矛盾。

格式约定（抓取端解析失败即视为缺陷，e2e 逐条断言）：
- 每个指标都带 `# HELP` / `# TYPE`；**没有样本的族不输出**（避免空族）
- 标签值里的 `\`、`"`、换行按 exposition 规范转义
- 数值不使用科学计数法；非有限值写 `NaN` / `+Inf` / `-Inf`

未收到 `DispatcherState` 时，调度类指标输出 `0` 而不是缺样本（抓取端无需处理空洞）。

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

### 5.5 在线拓扑热生效（临时交通管制）

现场施工区、单向通行、临时标石过去只能进容器 `vim stations.yaml`，
既没有连通性校验，也没有对在途车辆的安全仲裁；而 `map_ingest` / Web 地图上传
会**整份覆盖**该文件，手改的管制会被静默抹掉。本节端点把这件事变成
"带校验、可预演、可回滚、原子生效"的运维动作。

**核心机制**：运行时**叠加层**（overlay）以 `enabled=false` / `bidirectional=false`
就地改目录副本，**不是**新地图版本——`map_hash` 原样拷贝。因此规划、骨架折线、
距离估算、时空窗/CBS 全部自动一致，而状态快照的版本门禁完全不受影响。

| 方法 | 路径 | 动作 |
|---|---|---|
| `GET` | `/api/v1/topology/overlay` | 读当前叠加层（`?yaml=1` 才回可编辑的 YAML） |
| `POST` | `/api/v1/topology/block_edge` | 封一条边（`action=block`） |
| `POST` | `/api/v1/topology/unblock_edge` | 解封一条边 |
| `POST` | `/api/v1/topology/one_way_edge` | 改成单向（`direction=forward|reverse`） |
| `POST` | `/api/v1/topology/two_way_edge` | 恢复双向 |
| `POST` | `/api/v1/topology/hot_update` | 全量替换叠加层（YAML 文本；`clear=true` 清空） |
| `POST` | `/api/v1/topology/preview` | 同 `hot_update`，强制 `dry_run=true`（绝不写盘、绝不改内存） |

#### 单条边请求体

```json
{
  "from_station": "b",
  "to_station": "c",
  "direction": "both",
  "operator_id": "op_zhang",
  "reason": "water main repair 2026-10-29",
  "expected_map_hash": "fnv1a64_...",
  "confirm": true,
  "allow_edge_under_vehicle": false
}
```

`operator_id` / `reason` / `confirm=true` **恒必填**（`dry_run` 也不例外）：
审计字段不能因为"只是预演"就不填，否则预演通过后真正提交时仍是同一份无主记录。
`expected_map_hash` 为空 = 不校验；生产应填（见"乐观并发"）。

#### 全量替换请求体

```json
{
  "overlay_yaml": "blocked_edges:\n  - from: \"b\"\n    to: \"c\"\n    reason: \"...\"\n",
  "operator_id": "op_zhang",
  "reason": "site-wide detour",
  "dry_run": false,
  "clear": false,
  "confirm": true
}
```

`overlay_yaml` 里**只写管制清单**即可，不必写 `base_map_hash`/`revision`/`updated_at`
——这三项由提交管线盖章。但未知字段仍然严格拒绝（两个位置：条目内部与顶层各报各的）。

#### 响应

```json
{
  "status": "ok",              // ok | noop | rejected
  "error_code": "ok",
  "message": "...",
  "resource_id": "route_edge:b__c",
  "impacted_vehicles": [],
  "impacted_missions": [],
  "unreachable_nodes": [],
  "reroute_failures": [],
  "rerouted": false,
  "catalog_map_hash": "fnv1a64_...",
  "overlay_revision": 2,
  "file_written": true
}
```

`status` 与 HTTP 码是**两个**字段：`noop`（幂等重放）也是 `200`，
但控制台据此区分"已生效"与"本来就是这样"。

#### `error_code` 词表与 HTTP 映射

这是 REST 与 ROS 服务的唯一真源（`TopologyErrorStatus`）；未知错误码一律 `500`，
绝不把"不认识的失败"伪装成 200/409。

| `error_code` | HTTP | 含义 / 处置 |
|---|---|---|
| `ok` | 200 | 已生效 |
| `no_change` | 200 | 规范化后与当前叠加层一致（**零写盘、零计数**，`revision` 不变） |
| `invalid_request` | 400 | 字段缺失/非法、边不存在于目录（`unknown_edge` 除外） |
| `not_confirmed` | 400 | `confirm` 不是 `true` |
| `unknown_edge` | 404 | 端点不在目录里（站点与路网节点都查） |
| `map_hash_mismatch` | 409 | `expected_map_hash` 与当前基础地图不符（乐观并发失败） |
| `would_disconnect_topology` | 409 | 提交后存在站点互不可达；回 `unreachable_nodes` |
| `edge_occupied_in_flight` | 409 | 在途车可能正压在该边上；回 `impacted_vehicles` |
| `would_strand_vehicles` | 409 | 有在途任务找不到替代路线；回 `reroute_failures` |
| `lock_conflict` | 409 | 需要的新路权被其它任务持有 |
| `overlay_invalid` | 409 | YAML 校验失败（未知键、自环、重复条目、既封又单向…） |
| `catalog_unavailable` | 503 | 目录尚未加载（地图还没进来） |
| `overlay_disabled` | 501 | 功能未启用：`topology_overlay_file` 为空。这是**部署配置**问题，重试无意义；`GET /api/v1/topology/overlay` 仍可读，但 `message` 会写明"disabled" |
| `overlay_persist_failed` | 500 | 叠加层落盘失败（内存**零改动**） |
| `internal_error` | 500 | 其它 |

#### 四条不可让渡的语义

1. **拒绝时不写盘。** 校验、连通性、在途影响、锁预检全部在内存里完成；
   磁盘写入是提交管线的**第一步**（见下），失败即整体不生效。
   e2e 对 `preview` 与 `400` 两条路径都断言"叠加层文件不存在"。
2. **写前持久化。** 提交顺序固定为：落盘 → 预约新锁 → 释放废弃锁 →
   为被封边加 `traffic_block:` 锁 → 解封已移除的边 → 换内存叠加层 →
   同步规划器 → 下发改道。进程在落盘之后任意时刻死掉，重启从冷启动回放
   就是目标态；死在落盘之前，哪里都没变。这免掉了"磁盘失败要回滚内存锁"
   这个最难写对的分支。
3. **在途车压在边上 -> 默认拒绝。** `vehicle_safe_nodes_` 来自 VDA 5050
   `lastNodeId`，只说明"最后确认到达的节点"——车可能正驶在下一段上。
   若某车剩余航段的**第一段**被封，判 `kOnBlockedEdge` 并整单 409。
   `allow_edge_under_vehicle=true` 时改为"对**新**规划立刻生效，物理通行等该车
   自己驶离"，此时该边的锁留给在途车（跳过 `Block`，计入 `retained_edge_locks`）。
   **单向反转不提供 force**：让在途车反向穿越没有安全解，一律 409。
4. **基础地图换版即失效。** 叠加层记录的 `base_map_hash` 与当前目录不符时，
   文件被改名归档为 `<path>.stale.<epoch>`、内存叠加层清空、审计记
   `topology_overlay_invalidated` 并发事件（供告警 `AmrTopologyOverlayActive` 之外的
   "临时改道被静默清掉"场景）。**绝不**把施工区套到重新出图后的新地图上。

#### 已知语义缺口：单向在预约层没有表达形式

`route_edge` 锁是**无序**互斥（`route_edge:A__B` 与 `B__A` 同键），它的职责是
"两车不得同时占用该段"，与方向无关。**单一只由规划层保证**
（`bidirectional=false`）。因此操作员在 `list_traffic_reservations` /
`/api/v1/state` 里看不到"这条边现在是单向"——请看
`/api/v1/topology/overlay` 的 `one_way_edges` 或 `DispatcherState.one_way_edge_count`。

#### 与 `/v2/block_station_route` 的区别

两者并存，语义不同，**不要互换**：

| | `/v2/block_station_route`（旧） | `/api/v1/topology/*`（新） |
|---|---|---|
| 改规划 | 否（规划层仍认为路可走） | 是（`enabled`/`bidirectional`） |
| 改预约表 | 是（加 `traffic_block:` 锁） | 是（第二道防线） |
| 连通性校验 | 无 | 有（拒绝断连） |
| 在途仲裁 | 无 | 有（改道或拒绝） |
| 持久化 | 无（重启即失） | 有（叠加层文件 + 审计） |

---

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
| topology_panel（相位 11） | 就绪四组件表渲染 + 叠加层计数 / revision 显示 |
| topology_preview_reject（相位 11） | 断连叠加层预演被拒（`unreachable_nodes` 非空）、**应用按钮保持 disabled**、且未写盘 |
| topology_block（相位 11） | 单边封路成功 → 计数变化 + 文本框读回含该边 |
| readiness_degraded（相位 11） | dispatcher 停止 → 该行转红且页面继续轮询（证明 503 的 body 被渲染，不是只看状态码） |

环境隔离：ROS 栈在容器内跑独立域（`ROS_DOMAIN_ID=82` / MQTT :18890 / REST :18092），
浏览器在宿主机通过容器 IP 直连网关。
