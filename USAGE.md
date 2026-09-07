# AMR Dispatcher 完整使用与运行操作手册

本文档详细说明 `amr_dispatcher` 项目的环境依赖、两种运行模式（轻量核心模式与完整 ROS 2 模式）、四种交互方式、配置文件修改及常见问题排查。

---

## 一、项目架构与环境要求

### 1.1 核心技术栈
- **编程语言**：C++17 / C++20
- **中间件与框架**：ROS 2 Jazzy, ros2_control
- **依赖库**：`yaml-cpp`, `GoogleTest`, `pthreads`
- **容器环境**：Docker & Docker Compose

### 1.2 目录结构速览
```
amr_dispatcher/
├── config/                  # 业务与硬件配置文件 (chassis/safety/dispatcher/stations)
├── docker/                  # Dockerfile 与 docker-compose.yml
├── docs/                    # 架构规范、技术决策(ADR)、API字典、路线图
├── launch/                  # 系统启动脚本 (full_system, core_only, bringup, ros2_control)
├── scripts/                 # 自动化脚本 (build, run_demo, benchmark, record_topics, validate_config)
├── src/
│   ├── amr_dispatcher_core/        # 核心算法与协议库 (纯 C++, 零 ROS 依赖)
│   ├── amr_dispatcher_interfaces/  # 自定义 ROS 2 msg/srv/action
│   ├── amr_dispatcher_ros/         # ROS 2 运行时节点与 ros2_control 硬件插件
│   └── amr_dispatcher_tools/       # CLI 诊断客户端与 REST HTTP 网关
├── test/
│   ├── unit/                # 19 个单元测试套件
│   └── integration/         # 端到端全链路集成测试
├── CMakeLists.txt           # 顶层工作区 CMake
├── USAGE.md                 # 本操作手册
└── README.md                # 项目简介
```

---

## 二、模式 A：宿主机核心模式（极速开发、单测与压测）

适用于算法迭代、协议调试、性能评测。**此模式完全脱离 ROS 2 运行时，无需安装或启动 ROS 守护进程。**

### 2.1 编译核心库并运行全部测试（推荐首选验收）
```bash
cd ~/amr_dispatcher
./scripts/run_demo.sh
```
**运行内容**：
- 自动完成 `amr_dispatcher_core` 的 CMake 配置与 Release 编译。
- 自动运行全部 **21 个测试套件，共 130 个单元与集成测试用例（100% Passed）**。
- 典型运行时间约为 **4 秒**。

若需要单独构建核心库：
```bash
cmake -S src/amr_dispatcher_core -B build/core -DBUILD_TESTING=ON -DCMAKE_CXX_FLAGS="-std=c++20"
cmake --build build/core -j$(nproc)
ctest --test-dir build/core --output-on-failure
```

### 2.2 运行性能基准极限压测
```bash
./scripts/benchmark.sh
```
**压测指标**：
- **任务队列**：10,000 个任务高频入队与出队打分（实测吞吐约 **25,000 ops/s**）。
- **异步日志**：50,000 条日志高频写入（实测吞吐约 **2,000,000 ops/s**，完全无锁入队，不阻塞主控制循环）。

### 2.3 校验 YAML 配置文件合法性
```bash
python3 scripts/validate_config.py config/*.yaml config/scenarios/*.yaml
```

---

## 三、模式 B：完整 ROS 2 系统运行（Docker 一键拉起）

适用于全系统联调、ROS 2 节点通信、CLI 交互提单与外部 WMS/MES REST API 对接。

### 3.1 一键构建并启动全系统
```bash
cd ~/amr_dispatcher/docker
docker compose up --build
```
启动后，容器内自动编译全部 4 个 ROS 2 子包，并执行 `full_system.launch.py`，拉起以下 6 个核心服务：
1. **`dispatcher_node`** (Lifecycle 节点)：处理任务优先级队列、资源互斥原子锁、死锁巡检，并提供 `/dispatcher/execute_mission` Action 动作服务。
2. **`safety_gate_node`**：多源安全仲裁（急停、人工接管、碰撞杠、软看门狗超时），输出限速或停止指令。
3. **`chassis_driver_node`**：驱动底盘硬件通信、丢包监控、多级自动降级、发布 `/odom` 与 TF。
4. **`path_tracker_node`**：Pure Pursuit / Stanley 控制器，跟踪 `/plan` 路径并输出速度。
5. **`rest_gateway`**：监听在 `8080` 端口的轻量 HTTP REST 提单网关与静态 Web 页面服务。
6. **`dispatcher_visualizer_node`**：向 RViz2 发布 3D 拓扑路网、绿色占用锁与红色死锁报警 Marker。

---

## 四、系统交互与日常使用

系统启动后，可通过以下 5 种方式与调度与控制系统交互：

### 4.0 浏览器 Web 运维控制台 (`operator_console.html`)（推荐首选）
系统启动后，在任何主机的浏览器中访问：
```text
http://127.0.0.1:8080/
# 或者直接双击本地打开 tools/operator_console.html
```
**功能特性**：
- **调度大盘**：实时查看任务队列深度、当前生效调度算法、死锁警戒灯。
- **安全监控**：实时查看急停状态、看门狗喂狗心跳、底盘通信后端（Serial/UDP/Mock）与滑动窗口丢包率。
- **动态提单**：通过 UI 表单点选起点/终点与优先级，一键提单下发。
- **一键急停**：右上角醒目的红色 `🚨 EMERGENCY STOP` 按钮，瞬间熔断系统输出。
- **在线切换算法**：在下拉框中选择 `EarliestDeadlineFirst` 或 `WeightedScore`，点击即可无缝在线热更调度算法。

### 4.1 交互终端客户端 (`dispatcher_cli`)
进入正在运行的容器（或新起终端）：
```bash
docker exec -it docker-amr_dispatcher-1 bash
source /opt/ros/jazzy/setup.bash
source /workspace/install/setup.bash

# 运行交互命令行客户端
ros2 run amr_dispatcher_tools dispatcher_cli
```

**支持的命令示例**：
```text
# 1. 查看当前调度器状态（策略、队列深度、活跃任务、死锁标记）
amr_dispatcher> status
=== Dispatcher State ===
 Comparator:  priority_fifo
 Queue:       0/128
 Active:      0
 Deadlock:    NO

# 2. 提交新任务：格式为 submit <mission_id> <from_station> <to_station> <priority>
amr_dispatcher> submit task_001 station_pickup_A station_dropoff_1 10
[SUCCESS] Submitted mission task_001: Mission accepted

# 3. 暂停正在排队或执行的任务/订单
amr_dispatcher> pause task_001
[RESULT] Paused order/mission: task_001

# 4. 恢复已暂停的任务
amr_dispatcher> resume task_001
[RESULT] Resumed order/mission: task_001

# 5. 取消任务
amr_dispatcher> cancel task_001
[RESULT] Mission canceled from queue (canceled count=1)

# 6. 退出 CLI
amr_dispatcher> exit
```

---

### 4.2 HTTP REST API（供 WMS / MES 上层业务系统对接）
HTTP 服务默认监听在宿主机 `http://127.0.0.1:8080`：

#### 1. 查询系统全局状态
```bash
curl -s http://127.0.0.1:8080/api/v1/state | jq
```
**响应示例**：
```json
{
  "comparator": "priority_fifo",
  "queue_size": 2,
  "queue_capacity": 128,
  "active_count": 1,
  "deadlock": false
}
```

#### 2. 提交运单（支持幂等性令牌防重机制）
```bash
curl -s -X POST "http://127.0.0.1:8080/api/v1/orders?id=order_1001&from=station_pickup_A&to=station_dropoff_1&prio=20&idempotency_key=unique_token_999" | jq
```
**响应示例**：
```json
{
  "status": "ok",
  "accepted": true,
  "mission_id": "order_1001"
}
```
*注：携带相同 `idempotency_key` 的重复请求会被自动识别并幂等忽略，返回 `{"status":"ignored","reason":"idempotent_duplicate"}`。*

---

### 4.3 原生 ROS 2 命令行交互
在容器内部，使用标准 ROS 2 工具直接观察与调用：

#### 1. 查看话题与节点
```bash
ros2 node list
ros2 topic list
```

#### 2. 实时观察底盘通信健康度（丢包率、延迟统计）
```bash
ros2 topic echo /chassis/link_health
```

#### 3. 模拟触发急停信号（测试多源安全仲裁机制）
```bash
# 触发急停
ros2 topic pub /safety/estop std_msgs/msg/Bool "data: true" --once

# 观察安全门输出（应显示 motion_allowed: false, severity: CRITICAL, 速度全停）
ros2 topic echo /safety/state --once

# 恢复急停
ros2 topic pub /safety/estop std_msgs/msg/Bool "data: false" --once
```

#### 4. 通过 ROS 2 Service 申请交通管制锁
```bash
ros2 service call /dispatcher/reserve_resource amr_dispatcher_interfaces/srv/ReserveResource \
  "{resource_id: 'route_edge:station_pickup_A->station_dropoff_1', holder_id: 'robot_01', timeout_ms: 30000}"
```

#### 5. 12 个核心 ROS 2 Service (/v2/*) 操作示例
```bash
# 1. 任务入队
ros2 service call /v2/enqueue_mission amr_dispatcher_interfaces/srv/EnqueueMission \
  "{mission_file: 'patrol_dock_a', priority: 10, start_if_idle: true}"

# 2. 任务取消
ros2 service call /v2/cancel_queued_mission amr_dispatcher_interfaces/srv/CancelQueuedMission \
  "{mission_id: 'm_1', cancel_active: true}"

# 3. 任务紧急抢占
ros2 service call /v2/preempt_mission amr_dispatcher_interfaces/srv/PreemptMission \
  "{mission_file: 'urgent_transport', priority: 120}"

# 4. 调整排队优先级
ros2 service call /v2/reprioritize_queued_mission amr_dispatcher_interfaces/srv/ReprioritizeQueuedMission \
  "{mission_id: 'm_2', priority: 50}"

# 5. 任务成本与电量前置估算
ros2 service call /v2/estimate_mission_cost amr_dispatcher_interfaces/srv/EstimateMissionCost \
  "{mission_file: 'patrol_loop', nominal_speed_mps: 0.5, battery_voltage: 24.0}"

# 6. 人工临时封路
ros2 service call /v2/block_station_route amr_dispatcher_interfaces/srv/BlockStationRoute \
  "{from_station: 'station_pickup_A', to_station: 'station_pickup_B', reason: 'maintenance'}"

# 7. 主动死锁环路诊断
ros2 service call /v2/detect_traffic_deadlock amr_dispatcher_interfaces/srv/DetectTrafficDeadlock "{}"

# 8. 查询所有拓扑站点
ros2 service call /v2/list_stations amr_dispatcher_interfaces/srv/ListStations "{}"

# 9. 查询全网所有交通锁占用
ros2 service call /v2/list_traffic_reservations amr_dispatcher_interfaces/srv/ListTrafficReservations "{}"

# 10. 物理设施资源预约 (门/电梯/充电桩)
ros2 service call /v2/reserve_facility_resource amr_dispatcher_interfaces/srv/ReserveFacilityResource \
  "{resource_id: 'auto_door_1', holder_id: 'robot_01', mission_id: 'm_1', release_existing_for_holder: false}"

# 11. 获取全局运营快照大盘
ros2 service call /v2/get_operator_snapshot amr_dispatcher_interfaces/srv/GetOperatorSnapshot "{event_limit: 10}"

# 12. 校验站点配置一致性
ros2 service call /v2/validate_site_config amr_dispatcher_interfaces/srv/ValidateSiteConfig \
  "{bundle_dir: 'config', version_id: 'v1.1.0'}"
```

---

### 4.4 诊断话题录包 (`record_topics.sh`)
在容器内可一键记录所有状态与通信健康话题，生成 rosbag 用于问题复盘：
```bash
./scripts/record_topics.sh /tmp/diagnostics_bag
```

---

## 五、系统参数配置说明

全部运行参数位于 `config/` 目录下，修改对应 YAML 即可直接生效：

| 配置文件 | 关键参数项 | 说明 |
|---|---|---|
| **`config/chassis.yaml`** | `serial_device` | 物理串口设备路径（如 `/dev/ttyUSB0`） |
| | `serial_baud` | 通信波特率（默认 `115200`） |
| | `demote_loss_rate` | 触发硬件自动降级的丢包率阈值（默认 `0.30` 即 30%） |
| | `initial_backoff_ms` | 重连自愈初始退避毫秒数（默认 `200`） |
| **`config/dispatcher.yaml`** | `comparator` | 任务调度排序策略：`priority_fifo` / `earliest_deadline` / `shortest_job` / `weighted_score` |
| | `queue_capacity` | 调度任务队列最大容量（默认 `128`） |
| | `active_idle_threshold_seconds`| 判定任务假死卡顿的超时阈值（默认 `10` 秒） |
| **`config/safety.yaml`** | `max_linear_x_mps` | 最大前进线速度限制（默认 `1.5` m/s） |
| | `heartbeat_timeout_ms` | 软件看门狗心跳超时预警周期（默认 `500` ms） |
| **`config/stations.yaml`** | `stations` 列表 | 定义各工位站点的 ID、坐标 `(x, y, yaw)` 与类型（pickup/dropoff/charge） |

---

## 六、常见问题排查 (Troubleshooting)

### Q1: 在宿主机运行 `./scripts/build.sh` 提示没有 ROS 2 Jazzy？
- **正常现象**。宿主机若为 CentOS 或未安装 Jazzy 的环境，`build.sh` 会自动仅构建并测试 `amr_dispatcher_core` 纯 C++ 库（确保 130 个用例全过）。完整的 ROS 2 节点请通过 `docker compose up --build` 运行。

### Q2: 真实小车连接后底盘驱动降级为 MockBackend？
- 检查 `config/chassis.yaml` 中的 `serial_device` 设备权限（通常需要 `sudo chmod 666 /dev/ttyUSB0` 或加入 `dialout` 用户组）。
- 若无法连接物理串口，控制器将平滑降级至 `MockBackend` 保证调度逻辑不崩溃，属于预期保护机制。

### Q3: 任务一直处于 PENDING 无法激活？
- 检查 `config/dispatcher.yaml` 中的 `max_active_missions` 配置，若当前活跃任务已满，新任务将在队列排队等待前序任务完成。
