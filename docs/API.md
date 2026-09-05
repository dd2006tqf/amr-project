# AMR Dispatcher API Specification

## 1. ROS 2 话题接口 (Topics)

### 发布话题 (Published Topics)
| 话题名称 | 消息类型 | 频率 | 描述 |
|---|---|---|---|
| `/odom` | `nav_msgs/msg/Odometry` | 50 Hz | 底盘里程计反馈（位置、姿态四元数、线速度、角速度） |
| `/chassis/link_health` | `amr_dispatcher_interfaces/msg/ChassisLinkHealth` | 2 Hz | 底盘硬件通信链路健康指标（丢包率、CRC错、RTT时延） |
| `/dispatcher/state` | `amr_dispatcher_interfaces/msg/DispatcherState` | 2 Hz | 调度器全局状态（当前策略、队列深度、活跃任务列表、死锁状态） |
| `/dispatcher/events` | `amr_dispatcher_interfaces/msg/MissionEvent` | 事件触发 | 任务生命周期状态跃迁审计事件 |
| `/cmd_vel_safe` | `geometry_msgs/msg/Twist` | 50 Hz | 经过安全门控仲裁后的最终安全控制速度（下发至底盘驱动） |
| `/safety/state` | `amr_dispatcher_interfaces/msg/SafetyState` | 10 Hz | 安全门控运行状态（是否允许运动、当前限速比例、阻断源） |
| `/tracking/error` | `amr_dispatcher_interfaces/msg/TrackingError` | 20 Hz | 循迹跟踪误差（横向误差、航向误差、前视距离、到达标记） |
| `/system/aggregated_link_health` | `amr_dispatcher_interfaces/msg/ChassisLinkHealth` | 2 Hz | 聚合系统多链路的整体健康评级 |

### 订阅话题 (Subscribed Topics)
| 话题名称 | 消息类型 | 描述 |
|---|---|---|
| `/cmd_vel_raw` | `geometry_msgs/msg/Twist` | 循迹控制器或遥控节点发出的原始期望速度 |
| `/safety/estop` | `std_msgs/msg/Bool` | 紧急停止按钮状态输入（true 触发全停） |
| `/safety/bumper` | `std_msgs/msg/Bool` | 碰撞保险杠接触信号输入（true 触发全停） |
| `/safety/heartbeat` | `std_msgs/msg/Bool` | 外部节点或驱动的心跳保活信号 |
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

---

## 4. REST HTTP 网关接口 (REST API)

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
- **Response (Success)**:
```json
{
  "status": "ok",
  "accepted": true,
  "mission_id": "task_1001"
}
```
