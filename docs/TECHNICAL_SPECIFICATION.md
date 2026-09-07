# AMR Dispatcher 系统深化与改造技术实现方案 (Technical Implementation Specification)

> **文档性质**：指导下一步系统重构与编码落地的核心技术规范与开发文档（Technical Spec）。  
> **指导原则**：
> 1. **业务与中间件严格正交**：`amr_dispatcher_core` 保持纯 C++17/20 零 ROS 依赖；ROS 2 作为工业级中间件（Lifecycle、Action、QoS、Diagnostics、Params、Markers）深度落地。
> 2. **算法接口标准化占位（IoC / Strategy Pattern）**：定义清晰的 Planner 与 Tracker 纯虚基类，当前提供轻量基准/Mock 实现保证系统全链路闭环，预留未来平滑扩展槽位。
> 3. **吸收 `robot-project` 工业级最佳实践**：严密借鉴其拓扑锁唯一命名律、三级安全门控制模式仲裁、多维健康状态聚合到急停闭环、3D Marker 拓扑语义分层渲染，以及轻量单文件 Web 控制台。

---

## 目录
- [一、 模块改造架构概览与源工程对比](#一-模块改造架构概览与源工程对比)
- [二、 深入借鉴 robot-project 的核心技术架构与设计模式](#二-深入借鉴-robot-project-的核心技术架构与设计模式)
  - [2.1 拓扑资源锁与冲突死锁决策引擎 (借鉴 `robot_tasks/traffic_reservation`)](#21-拓扑资源锁与冲突死锁决策引擎-借鉴-robot_taskstraffic_reservation)
  - [2.2 多源分级安全门与控制模式仲裁 (借鉴 `robot_teleop/cmd_vel_safety_gate_node`)](#22-多源分级安全门与控制模式仲裁-借鉴-robot_teleopcmd_vel_safety_gate_node)
  - [2.3 状态监测与急停故障闭环 (借鉴 `robot_utils/system_monitor` 与 `fault_supervisor`)](#23-状态监测与急停故障闭环-借鉴-robot_utilssystem_monitor-与-fault_supervisor)
  - [2.4 3D 拓扑语义分层渲染与生命周期 (借鉴 `robot_simulation/amr_sim_visualizer_node`)](#24-3d-拓扑语义分层渲染与生命周期-借鉴-robot_simulationamr_sim_visualizer_node)
  - [2.5 单文件零依赖运营控制台与快照轮询机制 (借鉴 `tools/operator_console.html`)](#25-单文件零依赖运营控制台与快照轮询机制-借鉴-toolsoperator_consolehtml)
- [三、 算法占位接口体系设计（IoC 纯虚基类与 Mock 实现）](#三-算法占位接口体系设计ioc-纯虚基类与-mock-实现)
  - [3.1 路径规划器抽象：`IPathPlanner` 与 `MockLinearPlanner`](#31-路径规划器抽象ipathplanner-与-mocklinearplanner)
  - [3.2 路径跟踪控制器抽象：`IPathTracker`](#32-路径跟踪控制器抽象ipathtracker)
- [四、 ROS 2 中间件协议与 Lifecycle 体系规范](#四-ros-2-中间件协议与-lifecycle-体系规范)
  - [4.1 接口扩展：`ExecuteMission.action` 与 `TopologyState.msg`](#41-接口扩展executemissionaction-与-topologystatemsg)
  - [4.2 调度节点全生命周期改造：`dispatcher_lifecycle_node.cpp`](#42-调度节点全生命周期改造dispatcher_lifecycle_nodecpp)
  - [4.3 动态参数配置中心与在线热更回调](#43-动态参数配置中心与在线热更回调)
- [五、 可观测性与交互控制面技术落地](#五-可观测性与交互控制面技术落地)
  - [5.1 RViz2 拓扑语义 MarkerArray 渲染器：`dispatcher_visualizer_node.cpp`](#51-rviz2-拓扑语义-markerarray-渲染器dispatcher_visualizer_nodecpp)
  - [5.2 增强型 REST 网关与快照聚合路由：`rest_gateway.cpp`](#52-增强型-rest-网关与快照聚合路由rest_gatewaycpp)
  - [5.3 Web 控制台无缝对接：`operator_console.html`](#53-web-控制台无缝对接operator_consolehtml)
- [六、 混沌工程与端到端自动化验收脚本体系](#六-混沌工程与端到端自动化验收脚本体系)
- [七、 实施进度计划与文件修改清单](#七-实施进度计划与文件修改清单)

---

## 一、 模块改造架构概览与源工程对比

通过对 `robot-project` 源代码（21 个 ROS 2 子包、46 个业务源文件、110+ 脚本）的全面解构，结合我们“核心算法纯解耦、中间件规范化”的定位，改造后的项目目录如下：

```text
amr_dispatcher/
├── src/
│   ├── amr_dispatcher_core/                  # 纯 C++17/20 核心领域库 (零 ROS 依赖)
│   │   ├── include/amr_dispatcher_core/
│   │   │   ├── planning/                     # [新增] 算法占位接口抽象层
│   │   │   │   ├── path_planner_interface.hpp
│   │   │   │   └── mock_linear_planner.hpp
│   │   │   ├── path_tracking/                # [重构] 继承标准 IPathTracker 接口
│   │   │   │   ├── path_tracker_interface.hpp
│   │   │   │   ├── pure_pursuit_tracker.hpp
│   │   │   │   └── stanley_tracker.hpp
│   │   │   ├── dispatcher/                   # [扩展] 吸收规范的 RouteLock 命名律与死锁判定
│   │   │   │   ├── traffic_reservation.hpp
│   │   │   │   └── facility_reservation.hpp  # [新增] 充电桩/自动门租约锁抽象
│   │   │   ├── safety/                       # 仲裁决策纯逻辑
│   │   │   └── chassis/                      # 硬件抽象与滑动窗口丢包降级
│   │   └── test/                             # 保持 ~4s 100% 极速单测覆盖
│   ├── amr_dispatcher_interfaces/            # ROS 2 接口定义
│   │   ├── action/
│   │   │   └── ExecuteMission.action         # [新增] 异步长任务调度与流式进度/取消协议
│   │   └── msg/
│   │       ├── TopologyState.msg             # [新增] 拓扑边锁与死锁状态聚合
│   │       └── SafetyState.msg               # [新增] 对齐工业多源安全状态
│   ├── amr_dispatcher_ros/                   # ROS 2 Jazzy 适配中间件层
│   │   ├── src/
│   │   │   ├── dispatcher_lifecycle_node.cpp # [升级] rclcpp_lifecycle + Action Server + Params 热更
│   │   │   ├── safety_gate_node.cpp          # [重构] 对齐三级控制模式仲裁与 Watchdog 保护
│   │   │   ├── fault_supervisor_node.cpp     # [新增] 借鉴系统健康闭环至急停服务调用
│   │   │   ├── system_monitor_node.cpp       # [新增] diagnostic_updater 状态聚合
│   │   │   └── dispatcher_visualizer_node.cpp# [新增] RViz2 3D MarkerArray 语义渲染
│   └── amr_dispatcher_tools/
│       └── src/
│           └── rest_gateway.cpp              # [增强] 支持 /api/operator/* 聚合快照与静态页面托管
├── tools/
│   └── operator_console.html                 # [搬运定制] 零依赖工业级 Web 运维控制台
└── scripts/                                  # [新增] 混沌工程自动化验收流水线
    ├── check_deadlock_resolution.sh
    ├── check_hardware_failover.sh
    ├── check_watchdog_estop.sh
    └── check_all.sh
```

---

## 二、 深入借鉴 robot-project 的核心技术架构与设计模式

### 2.1 拓扑资源锁与冲突死锁决策引擎 (借鉴 `robot_tasks/traffic_reservation`)

在 `robot-project/src/robot_tasks/src/traffic_reservation.cpp` 中，其资源锁管理体现了极其优良的工业确定性设计模式：

1. **规范的资源 ID 归一化命名律 (Canonical Resource ID Naming)**：
   - **双向无向边锁归一化**：
     ```cpp
     std::string RouteLockId(const std::string& from, const std::string& to) {
       return from <= to ? "route_edge:" + from + "__" + to 
                         : "route_edge:" + to + "__" + from;
     }
     ```
     **设计妙处**：不管 AMR 是从 A 走向 B，还是从 B 走向 A，生成的锁 ID 都是唯一的 `route_edge:A__B`，彻底消除了字符串匹配漏洞导致的并发幽灵死锁。
   - **节点锁格式**：`"route_node:" + station_id`。
   - **临时施工/故障封路锁**：`"traffic_block:" + reason`，支持运营管理员通过封路锁动态禁用局部路网。
2. **多车时空预约区间锁（Time-Space Reservation Table）**：
   - 彻底超越简单的单队列或静态互斥锁，引入基于时间窗口的区间预约 $[t_{\text{start}}, t_{\text{end}}]$。
   - 允许多台 AMR 在不同时段错峰共享同一条物理路段，只要时间窗口不重合即可并行放行；若发生时间区间重叠（Overlap），立即拦截并反馈冲突，由调度器插入等待窗口。
3. **图论等待图（Wait-For Graph）深度优先环检测算法**：
   - 不再依赖低级的 30 秒超时启发式“猜死锁”；
   - 动态构建资源申请图：任务 $M_1 \to$ 等待资源 $R \to$ 持有者 $M_2$；
   - 采用 DFS 三色标记拓扑探查，**在资源锁申请被阻塞的瞬间（0 毫秒）即时判定成环**，并输出完整冲突成环路径 `[M1 -> M2 -> M1]`，依据任务优先级实施确定性回滚与抢占让行！

### 2.2 多源分级安全门与控制模式仲裁 (借鉴 `robot_teleop/cmd_vel_safety_gate_node`)

在 `robot-project/src/robot_teleop/src/cmd_vel_safety_gate_node.cpp` 中，速度指令不是简单地直通底盘，而是构建了**工业级三层控制仲裁体系**：

```
上层输入源:
  [/tracking_cmd_vel] (自主路径跟踪)
  [/teleop_cmd_vel]   (人工遥控接管)
           │
           ▼
┌────────────────────────────────────────────────────────────────────────┐
│                      cmd_vel_safety_gate_node                          │
│                                                                        │
│  [优先级 1: 硬/软 E-Stop 锁定] ─────────► 强制 Twist(0,0)，切断输出     │
│  [优先级 2: 人工遥控接管 (Manual Takeover)] ──► 压制自主跟踪，放行遥控 │
│  [优先级 3: 软看门狗超时 (Watchdog Timeout)] ─► 丢失心跳强制紧急刹车   │
│  [优先级 4: 动态区域限速 (Zone Speed Limit)] ──► 速度平滑饱和截断 (Clamp)│
└───────────────────────────────────┬────────────────────────────────────┘
                                    │
                                    ▼
                          [/cmd_vel] ──► 底盘驱动
```

- **安全仲裁关键参数与逻辑**：
  - `watchdog_timeout_ms`（默认 200~500ms）：若上一级控制器在超时时间内未更新指令，立刻触发制动，发布 `/cmd_vel` 全零，并在状态中标记 `WATCHDOG_TIMEOUT`。
  - `manual_takeover`（人工接管标志）：运维人员在 Web 或手柄按下遥控键时，自动锁死并丢弃自动调度下发的速度，保证人身安全绝对优先。
  - `effective_speed_limit_mps`：当底盘上报链路丢包、或通过危险狭窄区域时，将线性速度安全截断：`linear_x = std::clamp(cmd.linear_x, -limit, limit)`。

### 2.3 状态监测与急停故障闭环 (借鉴 `robot_utils/system_monitor` 与 `fault_supervisor`)

`robot-project` 实现了非常标准的可观测性到安全闭环体系：
1. **`SystemMonitorNode` (信息采集与聚合)**：
   - 依赖 ROS 2 标准的 `diagnostic_updater::DiagnosticUpdater`。
   - 周期性（200ms）轮询各子系统：
     - 底盘硬件状态（电压、温度、CRC 错误率）；
     - 传感器时效性（检查 `/scan`、`/imu`、`/odom` 的最后更新时间戳 `CheckTopicFreshness`）；
     - 将健康评估封装为 `robot_interfaces/msg/RobotState` 发布给 `/system_health`。
2. **`FaultSupervisorNode` (故障决策与执行中枢)**：
   - 订阅 `/system_health`。
   - **冷启动宽限保护（Startup Grace Period）**：系统启动的前 2000ms 内，即使传感器未就绪也只标记为 `ARMING`，避免因启动时差误触发急停。
   - **冷却防抖机制（Command Cooldown）**：在调用 `/enable_emergency_stop` 服务后，设置 500ms 冷却时间，防止在高频抖动时发生服务雪崩调用。
   - **自愈恢复闭环（Auto Recovery）**：当系统健康状态恢复 `OK` 且 `auto_clear:=true` 时，自动调用 `/clear_emergency_stop` 恢复常态运行。

### 2.4 3D 拓扑语义分层渲染与生命周期 (借鉴 `robot_simulation/amr_sim_visualizer_node`)

在 `robot-project/src/robot_simulation/src/amr_sim_visualizer_node.cpp` 中，RViz2 可视化采用了教科书级的 MarkerArray 组织方式：
- **命名空间隔离 (Namespace Separation)**：
  - `"station_routes"`：用 `Marker::LINE_LIST` 绘制基础静态路网拓扑。
  - `"station_labels"`：用 `Marker::TEXT_VIEW_FACING` 绘制带 3D 深度漂浮的文字。
  - `"traffic_reservations"`：用加粗动态变色折线表示已加锁路段。
  - `"traffic_deadlock"`：用红色半透明圆柱体 `Marker::CYLINDER` 标出冲突交汇点。
- **Marker 渲染生命周期保鲜（Lifetime Management）**：
  ```cpp
  marker.lifetime.sec = 1;
  marker.lifetime.nanosec = 200000000; // 1.2s 超时销毁
  ```
  **设计妙处**：每个 Marker 显式指定生命周期（1.2 秒）。这样即使网络抖动或节点重启，过期的死锁圆柱或锁线条会在 1.2 秒后由 RViz2 自动在画面中淡出消除，绝不会在屏幕上留下“幽灵残影”。
- **全量重置帧（`AddDeleteAll`）**：
  - 在每次重构路网时，队列首个元素发送 `marker.action = Marker::DELETEALL`，彻底清理客户端缓存。

### 2.5 单文件零依赖运营控制台与快照轮询机制 (借鉴 `tools/operator_console.html`)

`robot-project` 的 `operator_console.html` 提供了极高的交互质感与工程实用性：
- **无框架单文件设计**：纯原生 HTML5/CSS3/JavaScript（ES6），无需 `npm install`，双击浏览器即开。
- **快照式聚合轮询（Snapshot Polling）**：
  - 前端每 1000ms 异步请求一次 `${baseUrl}/api/operator/snapshot`。
  - 后端在一次响应中打包装载：
    - `robot_state` (模式、电量、当前坐标)；
    - `missions` (排队任务列表、当前执行中任务)；
    - `traffic_locks` (当前锁定路段列表、死锁状态标志位)；
    - `facilities` (充电桩/自动门占用情况)。
  - 前端据此局部刷新 DOM，提供高帧率仪表盘与操作按钮响应。

---

## 三、 算法占位接口体系设计（IoC 纯虚基类与 Mock 实现）

为确保项目不因算法缺席而破坏完整性，在 `amr_dispatcher_core` 中建立符合 SOLID 原则的纯虚接口规范。

### 3.1 路径规划器抽象：`IPathPlanner` 与 `MockLinearPlanner`

#### 接口定义：`src/amr_dispatcher_core/include/amr_dispatcher_core/planning/path_planner_interface.hpp`
```cpp
#pragma once

#include <cmath>
#include <string>
#include <vector>

namespace amr_dispatcher_core {

struct Point2D {
  double x{0.0};
  double y{0.0};
};

struct Pose2D {
  double x{0.0};
  double y{0.0};
  double theta{0.0}; // 航向角 (rad), 范围 [-M_PI, M_PI]
};

/**
 * @brief 路径规划器抽象基类 (IoC 接口)
 * 调度系统仅依赖此接口，不依赖任何具体规划算法。
 */
class IPathPlanner {
 public:
  virtual ~IPathPlanner() = default;

  /**
   * @brief 在起点与终点之间生成离散路径点序列
   * @param start 起点二维位姿
   * @param goal 终点二维位姿
   * @param edge_id 可选的拓扑边编号，供基于图先验规划使用
   * @return std::vector<Pose2D> 生成的路径序列；若失败返回空序列
   */
  virtual std::vector<Pose2D> Plan(const Pose2D& start, const Pose2D& goal,
                                   const std::string& edge_id = "") = 0;

  virtual std::string GetPlannerName() const = 0;
};

}  // namespace amr_dispatcher_core
```

#### 基准 Mock 实现：`src/amr_dispatcher_core/include/amr_dispatcher_core/planning/mock_linear_planner.hpp`
- **逻辑实现**：
  - 计算欧氏距离 $D = \sqrt{(x_g - x_s)^2 + (y_g - y_s)^2}$。
  - 航向角计算 $\theta = \text{atan2}(y_g - y_s, x_g - x_s)$。
  - 按照参数 `step_size`（默认 0.1m）进行等距几何插值，生成平滑线段。
  - 保证起点为序列第一个点，终点为序列最后一个点。
- **价值**：零外部依赖，1 微秒内完成路径合成，足以提供标准的 `nav_msgs::msg::Path` 用于可视化与底盘跟踪。
- **未来替换方案**：后续学习了 A* 或 Nav2 Costmap 之后，只需派生实现 `class AStarPlanner : public IPathPlanner`，直接依赖注入，调度器业务核心代码零改动。

### 3.2 路径跟踪控制器抽象：`IPathTracker`

#### 接口定义：`src/amr_dispatcher_core/include/amr_dispatcher_core/path_tracking/path_tracker_interface.hpp`
```cpp
#pragma once

#include <vector>
#include "amr_dispatcher_core/planning/path_planner_interface.hpp"

namespace amr_dispatcher_core {

struct TwistCommand {
  double linear_x{0.0};   // 期望线速度 (m/s)
  double angular_z{0.0};  // 期望角速度 (rad/s)
  bool reached_goal{false}; // 是否到达路径终点容差范围内
  double cross_track_error{0.0}; // 横向跟踪偏差 (m)
};

/**
 * @brief 路径跟踪控制器抽象基类
 */
class IPathTracker {
 public:
  virtual ~IPathTracker() = default;

  /**
   * @brief 根据当前位姿与目标路径计算速度控制量
   */
  virtual TwistCommand ComputeControl(const Pose2D& current_pose,
                                      const std::vector<Pose2D>& path) = 0;

  virtual void Reset() = 0;
  virtual std::string GetTrackerName() const = 0;
};

}  // namespace amr_dispatcher_core
```

- **重构现有控制器**：
  - 让 `PurePursuitTracker` 与 `StanleyTracker` 实现 `IPathTracker` 接口。
  - 调度节点仅需通过多态指针 `std::unique_ptr<IPathTracker> tracker_` 即可在 Pure Pursuit 与 Stanley 间无缝热切换。

---

## 四、 ROS 2 中间件协议与 Lifecycle 体系规范

### 4.1 接口扩展：`ExecuteMission.action` 与 `TopologyState.msg`

在 `src/amr_dispatcher_interfaces/` 下扩充关键定义：

#### 1. `action/ExecuteMission.action`
```text
# --- Goal: 任务提单目标 ---
string mission_id
uint8 priority                # 1 - 255 (高数值具有抢占优先权)
string target_station         # 目标站点标识 (如: "STATION_B")
float64 deadline_seconds      # 截止时间约束 (供 EDF 算法计算)
string[] required_resources   # 预申请的拓扑边或设施锁
---
# --- Result: 终态结果 ---
bool success
int32 error_code              # 0: OK, 1: RESOURCE_DEADLOCK, 2: TIMEOUT, 3: PREEMPTED, 4: HARDWARE_ERROR
string message
float64 total_duration_sec
---
# --- Feedback: 实时流式进度反馈 ---
uint8 state_code              # 0: QUEUED, 1: WAITING_LOCK, 2: EXECUTING, 3: YIELDING, 4: RECOVERING
string current_node
float32 percent_complete      # 0.0 - 100.0%
float64 estimated_remaining_sec
```

#### 2. `msg/TopologyState.msg`
```text
builtin_interfaces/Time stamp
string[] active_edges         # 路网中所有可通行的边
string[] locked_edges         # 当前已被占用的边 (例如: "route_edge:A__B")
string[] edge_owners          # 对应 locked_edges 的持有者任务 ID
string[] deadlocked_nodes     # 当前发生死锁环冲突的交叉节点
bool deadlock_detected        # 全局死锁布尔报警标志
```

#### 3. `msg/SafetyState.msg`
```text
std_msgs/Header header
bool estop_active             # 是否处于急停状态
bool watchdog_timeout         # 看门狗是否超时
bool manual_takeover          # 是否处于人工接管
float64 effective_speed_limit # 当前生效的最大安全速度截断值 (m/s)
string current_source         # 当前放行的速度指令源 ("tracking" / "teleop" / "stopped")
string message
```

### 4.2 调度节点全生命周期改造：`dispatcher_lifecycle_node.cpp`

将 `dispatcher_node` 重命名升级为 `DispatcherLifecycleNode`，继承自 `rclcpp_lifecycle::LifecycleNode`，实现四阶段严格受控状态转移：

```
                    [Unconfigured]
                          │ configure()
                          ▼
                     [Inactive] ◄──────────────┐
                          │ activate()         │ deactivate()
                          ▼                    │
                      [Active] ────────────────┘
                          │ cleanup()
                          ▼
                    [Unconfigured]
```

#### 阶段详细职责：
1. **`on_configure()`**：
   - 读取 YAML 配置文件，构建拓扑路网、站点表与内存分配。
   - 初始化 `MissionQueue`、`TrafficLockManager` 与 `MockLinearPlanner` 实例。
   - 实例化 Action Server `rclcpp_action::create_server<ExecuteMission>`。
   - 创建 Lifecycle Publishers（`/dispatcher/state`, `/dispatcher/topology_state`）。
   - 此时节点处于就绪状态，但**禁止处理 Action Goal，不广播有效控制指令**。
2. **`on_activate()`**：
   - 激活所有 `LifecyclePublisher`（调用 `on_activate()`）。
   - 启动 10Hz 调度主定时器 `schedule_timer_` 与 2Hz 死锁巡检定时器 `deadlock_timer_`。
   - 正式放行并接收外部 Action 提单。
3. **`on_deactivate()`**：
   - 挂起调度定时器，停止接单。
   - 对当前正在运行的所有 Action 执行 `handle_cancel()`，通知底盘优雅刹车。
   - 原子清空已分配的动态拓扑锁，调用 `LifecyclePublisher::on_deactivate()`。
4. **`on_cleanup()` / `on_shutdown()`**：
   - 释放核心指针，断开底层通信，重置状态至未初始化。

### 4.3 动态参数配置中心与在线热更回调

在 `DispatcherLifecycleNode` 构造器中声明关键业务参数，并挂载参数热更监听器：
```cpp
// 注册参数变更监听
param_callback_handle_ = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter>& params) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      for (const auto& param : params) {
        if (param.get_name() == "scheduling_strategy") {
          std::string strategy = param.as_string();
          if (core_dispatcher_->SetComparator(strategy)) {
            RCLCPP_INFO(get_logger(), "Hot-reloaded scheduling strategy to: %s", strategy.c_str());
          } else {
            result.successful = false;
            result.reason = "Unsupported strategy: " + strategy;
          }
        } else if (param.get_name() == "deadlock_timeout_ms") {
          int timeout = param.as_int();
          core_dispatcher_->SetDeadlockTimeoutMs(timeout);
          RCLCPP_INFO(get_logger(), "Hot-reloaded deadlock timeout to: %d ms", timeout);
        }
      }
      return result;
    });
```

---

## 五、 可观测性与交互控制面技术落地

### 5.1 RViz2 拓扑语义 MarkerArray 渲染器：`dispatcher_visualizer_node.cpp`

借鉴 `robot_simulation` 的成熟经验，创建专用可视化渲染节点：

```cpp
// 核心 Marker 构建逻辑速查
void DispatcherVisualizerNode::PublishMarkers(const TopologyState::SharedPtr state) {
  visualization_msgs::msg::MarkerArray markers;
  int id = 0;

  // 1. 全局重置清空旧缓存帧 (防幽灵 Marker 残留)
  markers.markers.push_back(CreateResetMarker(id++));

  // 2. 静态路网底图 (淡灰细线, LINE_LIST)
  markers.markers.push_back(CreateStaticRoutesMarker(id++));

  // 3. 动态路网占用锁 (高亮绿粗线, LINE_LIST)
  for (const auto& locked_edge : state->locked_edges) {
    auto edge_marker = CreateLockedEdgeMarker(id++, locked_edge);
    markers.markers.push_back(edge_marker);
  }

  // 4. 死锁冲突报警光柱 (红色半透明圆柱 + 3D 文字)
  if (state->deadlock_detected) {
    for (const auto& node_id : state->deadlocked_nodes) {
      markers.markers.push_back(CreateDeadlockCylinder(id++, node_id));
      markers.markers.push_back(CreateDeadlockText(id++, node_id, "DEADLOCK DETECTED"));
    }
  }

  marker_pub_->publish(markers);
}
```

- **QoS 策略**：发布话题 `/dispatcher/markers`，使用 `rclcpp::QoS(1).transient_local()`，保证打开 RViz2 时无需等待刷新，立即渲染全图。
- **RViz 配置文件**：在 `config/rviz/dispatcher_view.rviz` 中固化配置 `MarkerArray` 订阅与顶视角相机视角（Top-Down Orthogonal View）。

### 5.2 增强型 REST 网关与快照聚合路由：`rest_gateway.cpp`

将现有的 `rest_gateway` 升级，支持四大标准端点：

| HTTP 方法 | 请求路径 | 功能描述 | 响应示例 |
| :--- | :--- | :--- | :--- |
| **GET** | `/api/operator/snapshot` | 聚合返回全系统当前运行快照 | `{"status":"ACTIVE","strategy":"PriorityFIFO","queue_size":2,"active_missions":[{"id":"m1","state":"RUNNING"}],"locked_edges":["route_edge:A__B"],"deadlock":false}` |
| **POST** | `/api/operator/submit_order` | 外部提单，支持设置目标、优先级与截止时间 | `{"status":"ok","mission_id":"mission_101","accepted":true}` |
| **POST** | `/api/operator/cancel_order` | 主动取消/抢占正在执行的任务 | `{"status":"ok","canceled":true}` |
| **POST** | `/api/operator/estop` | 系统级软急停开关控制 | `{"status":"ok","estop_engaged":true}` |
| **GET** | `/` 或 `/index.html` | 内置静态 Web 文件路由 | 直接以 `text/html` 返回 `operator_console.html` 源码 |

### 5.3 Web 控制台无缝对接：`operator_console.html`

- 直接从 `../robot-project/tools/operator_console.html` 搬运至本项目的 `tools/operator_console.html`。
- 修改 JavaScript 中的轮询端点默认值为 `http://127.0.0.1:8080/api/operator/snapshot`。
- 页面提供：
  - **实时吞吐量与队列监控看板**；
  - **拓扑锁分布与死锁警报灯**；
  - **手动提交订单 Form 表单**；
  - **红色大按钮一键急停（Emergency Stop）**。

---

## 六、 混沌工程与端到端自动化验收脚本体系

在 `scripts/` 目录下构建自动化验收套件，形成项目交付与答辩的核心武器库：

### 6.1 `scripts/check_deadlock_resolution.sh`
- **验证目的**：验证当两台虚拟 AMR 对向行进造成环形死锁时，系统的死锁巡检机制能否在 2 秒内识别并强制解开死锁。
- **执行过程**：
  1. 调用 REST API 同时下发两笔冲突订单：Order1 (A -> B)，Order2 (B -> A)。
  2. 轮询 `/dispatcher/topology_state`，断言 `deadlock_detected == true`。
  3. 观察调度器决策日志：低优先级任务触发 `YIELDING`，释放持有的边锁并执行后退避让。
  4. 最终断言：高优先级任务顺利到达，低优先级任务随后重新获取锁并完成，退出码 `0`。

### 6.2 `scripts/check_watchdog_estop.sh`
- **验证目的**：验证控制节点崩溃时，Safety Gate 是否能在 200ms 内完成看门狗超时制动。
- **执行过程**：
  1. 启动全系统，让小车开始跟踪直线路径，监控 `/cmd_vel` 稳定输出线速度 $v = 0.5\text{ m/s}$。
  2. 获取 `path_tracker_node` 的 PID，发送暂停信号：`kill -STOP <PID>`。
  3. 在终端以 50Hz 采样 `/cmd_vel`，验证在 $200\text{ ms}$ 内，速度强制归零 ($v = 0.0, w = 0.0$)。
  4. 发送 `kill -CONT <PID>` 恢复进程，验证系统未崩溃。

### 6.3 `scripts/check_hardware_failover.sh`
- **验证目的**：验证串口硬件通信断开时，底盘驱动管理器的三级自动降级与丢包保护。
- **执行过程**：
  1. 系统默认以 `Serial` 后端运行。
  2. 运行脚本人为掐断虚拟串口数据流（注入 100% 丢包）。
  3. 读取 `/chassis/link_health`，验证丢包率滑动窗口统计超过阈值（30%）。
  4. 验证系统无缝降级转移：`Serial -> UDP -> Mock`，且 Safety Gate 自动执行降级限速（最高 0.2 m/s）。

### 6.4 `scripts/check_all.sh`（一键汇总交付脚本）
```bash
#!/usr/bin/env bash
set -e

echo "========================================================"
echo "      AMR Dispatcher 全系统工业级质量与混沌验收流水线    "
echo "========================================================"

echo "[1/5] 编译纯 C++ 核心库并执行 GoogleTest 单元测试..."
./scripts/build.sh

echo "[2/5] 执行核心库基准性能压测 (队列吞吐 & 异步日志)..."
./scripts/benchmark.sh

echo "[3/5] 验证并发死锁检测与自动抢占恢复机制..."
./scripts/check_deadlock_resolution.sh

echo "[4/5] 验证多源安全门看门狗 (Watchdog) 200ms 快速熔断..."
./scripts/check_watchdog_estop.sh

echo "[5/5] 验证底层硬件通信丢包三级容灾自动降级..."
./scripts/check_hardware_failover.sh

echo "========================================================"
echo "    [ALL CHECKS PASSED] 核心算法与中间件机制 100% 验证成功!   "
echo "========================================================"
```

---

## 七、 实施进度计划与文件修改清单

为了让后续编码工作严密推进，任务按以下 6 个步骤逐步执行：

| 步骤 | 阶段与任务目标 | 核心新建/修改文件 | 预期输出与验收方式 |
| :--- | :--- | :--- | :--- |
| **Step 1** | **算法接口抽象与 Mock 占位** | • `include/planning/path_planner_interface.hpp`<br>• `include/planning/mock_linear_planner.hpp`<br>• `include/path_tracking/path_tracker_interface.hpp`<br>• `test/unit/mock_linear_planner_test.cpp` | 运行 `./scripts/build.sh`，新增单测全绿，耗时维持 ~4s。 |
| **Step 2** | **搬运 Web 看板与增强 REST Gateway** | • `tools/operator_console.html`<br>• `src/amr_dispatcher_tools/src/rest_gateway.cpp` | 打开浏览器 `http://localhost:8080`，可正常看到状态看板并成功提单。 |
| **Step 3** | **RViz2 3D Marker 可视化节点开发** | • `src/amr_dispatcher_ros/src/dispatcher_visualizer_node.cpp`<br>• `config/rviz/dispatcher_view.rviz`<br>• `launch/full_system.launch.py` | 启动后打开 RViz2，能清晰看到路网、动态变色绿色占用锁及红色死锁圆柱。 |
| **Step 4** | **ROS 2 接口扩展与 Lifecycle 升级** | • `amr_dispatcher_interfaces/action/ExecuteMission.action`<br>• `amr_dispatcher_interfaces/msg/TopologyState.msg`<br>• `src/amr_dispatcher_ros/src/dispatcher_lifecycle_node.cpp` | 支持标准的 `ros2 lifecycle` 状态机切换与 `ros2 action send_goal` 提单。 |
| **Step 5** | **安全门多源仲裁与动态参数热重载** | • `src/amr_dispatcher_ros/src/safety_gate_node.cpp` | 支持 `ros2 param set` 实时在线切换调度策略；心跳丢失 200ms 内刹车。 |
| **Step 6** | **构建混沌演练自动化脚本矩阵** | • `scripts/check_deadlock_resolution.sh`<br>• `scripts/check_watchdog_estop.sh`<br>• `scripts/check_hardware_failover.sh`<br>• `scripts/check_all.sh` | 终端运行 `./scripts/check_all.sh`，5 个检验阶段全部通过输出绿标。 |
