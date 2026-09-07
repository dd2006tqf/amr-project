# AMR Dispatcher 架构演进与系统深化路线规划书

> **设计哲学定位**：**业务算法与通信中间件严格正交（Orthogonal Separation）**。
> 本项目定位为**工业级高可靠机器人边缘任务调度与分布式执行平台**。核心业务聚焦于确定性状态机、分布式资源互斥与防死锁、多源安全仲裁与容灾降级；ROS 2 则作为工业标准中间件，拉满其 Lifecycle 生命周期管理、Action 抢占通信、QoS 细粒度控制、参数热加载及诊断观测能力。
> **算法策略**：本阶段对运动规划与跟踪控制等数学算法做标准**占位设计（IoC / Strategy Pattern 接口抽象）**，提供轻量级几何/直线直连 Mock 实现保证全链路闭环，预留标准化算法扩展槽位，待后续学习深入后再平滑演进注入。

---

## 目录
- [一、 现状剖析与演进目标](#一-现状剖析与演进目标)
- [二、 算法模块占位符（IoC 接口体系）设计](#二-算法模块占位符ioc-接口体系设计)
- [三、 从 robot-project 借鉴与落地的核心系统机制](#三-从-robot-project-借鉴与落地的核心系统机制)
  - [1. 可观测性：RViz2 3D 拓扑路网与死锁渲染节点](#1-可观测性rviz2-3d-拓扑路网与死锁渲染节点)
  - [2. 控制面：单文件 Web 运营看板与 REST 桥接增强](#2-控制面单文件-web-运营看板与-rest-桥接增强)
  - [3. 安全机制：多源安全仲裁安全门与看门狗强化](#3-安全机制多源安全仲裁安全门与看门狗强化)
  - [4. 资源调度扩展：物理设施（充电桩/自动门）抽象锁](#4-资源调度扩展物理设施充电桩自动门抽象锁)
  - [5. 质量保障：场景化混沌演练与自动化验收脚本](#5-质量保障场景化混沌演练与自动化验收脚本)
- [四、 ROS 2 中间件技术栈深度落地规划](#四-ros-2-中间件技术栈深度落地规划)
  - [1. 节点生命周期（Lifecycle）全面改造](#1-节点生命周期lifecycle全面改造)
  - [2. 任务调度 Action 体系与抢占机制](#2-任务调度-action-体系与抢占机制)
  - [3. 工业级 QoS 策略与诊断系统接入](#3-工业级-qos-策略与诊断系统接入)
  - [4. 动态参数在线热重载（Dynamic Parameters）](#4-动态参数在线热重载dynamic-parameters)
- [五、 分阶段实施里程碑与时间表](#五-分阶段实施里程碑与时间表)
- [六、 答辩与面试展示全景剧本（5 分钟演示流）](#六-答辩与面试展示全景剧本5-分钟演示流)

---

## 一、 现状剖析与演进目标

### 1.1 现状基础
- **`amr_dispatcher_core`**：纯 C++17/20 实现，零 ROS 依赖，包含可插拔调度队列（FIFO/EDF/SJF/Score）、拓扑锁与死锁巡检、Chassis 丢包率滑动窗口监测与三级降级（Serial -> UDP -> Mock）、无锁异步日志。130 个 GoogleTest 单元测试 ~4 秒全通。
- **`amr_dispatcher_ros`**：基本的 ROS 2 驱动节点、安全门节点、调度节点与控制插件。
- **`amr_dispatcher_tools`**：基础命令行客户端与轻量 HTTP REST 提单网关。

### 1.2 演进目标
1. **视觉与交互升级**：告别黑漆漆的命令行日志，构建 **RViz2 3D 拓扑动态 Marker** 与 **Browser Web 运营控制台**，一秒展示调度、锁占用与死锁解除。
2. **中间件标准升级**：全面对齐 ROS 2 工业级规范（Lifecycle 状态机、Action 异步抢占通信、QoS 定制、参数热加载、Diagnostics 体系）。
3. **架构接口标准化（算法占位）**：抽离 Planner 与 Controller 纯虚接口，当前填充基准 Mock 实现，实现算法的“零侵入”后续接入。
4. **交付验证完备化**：建立混沌故障注入与端到端自动化验收 Shell 脚本矩阵。

---

## 二、 算法模块占位符（IoC 接口体系）设计

为防止未来引入复杂算法时破坏现有调度与系统架构，在 `amr_dispatcher_core` 中建立清晰的纯虚基类接口。

```
              ┌──────────────────────────────────────────────┐
              │           Mission Dispatcher (调度大脑)       │
              └──────────────────────┬───────────────────────┘
                                     │ 生成导航意图 / 经过站点序列
                                     ▼
        ┌──────────────────────────────────────────────────────────┐
        │               <<interface>> IPathPlanner                 │
        │  + PlanPath(start: Pose, goal: Pose, map: MapTopology)   │
        └────────────────────────────┬─────────────────────────────┘
                                     │ 输出: Path (有序 Pose 序列)
                    ┌────────────────┴────────────────┐
                    ▼ (当前占位实现)                   ▼ (未来算法槽位)
         [MockLinearPlanner]                 [A* / Hybrid A* Planner]
         • 坐标直线前向插值                    • 基于代价地图的避障路径
         • 零外部依赖，极速返回                • 复杂几何规划
                                     │
                                     ▼
        ┌──────────────────────────────────────────────────────────┐
        │               <<interface>> IPathTracker                 │
        │  + ComputeCommand(current: Pose, path: Path) -> CmdVel   │
        └────────────────────────────┬─────────────────────────────┘
                                     │ 输出: Twist (v, w)
                    ┌────────────────┴────────────────┐
                    ▼ (当前占位实现)                   ▼ (未来算法槽位)
         [GeometricTracker]                  [MPC / LQR Controller]
         • 基础 Pure Pursuit / Stanley        • 最优控制 / 动力学前馈
         • 初中几何纯追踪，轻量鲁棒            • 考虑底盘滑动与约束
                                     │
                                     ▼
              ┌──────────────────────────────────────────────┐
              │      Safety Gate Node (多源安全仲裁门)         │
              └──────────────────────────────────────────────┘
```

### 详细规范说明：
1. **`IPathPlanner`（路径规划接口）**：
   - 接口定义：`virtual std::vector<Pose2D> Plan(const Pose2D& start, const Pose2D& goal, const RouteGraph& graph) = 0;`
   - **当前占位实现 (`StraightLineMockPlanner`)**：
     - 根据拓扑图中的边（Edge），直接在起点与终点之间做等间距线性插值（如每 0.1m 插一个点）。
     - 作用：不依赖任何复杂的 A*、Nav2 Costmap，但能吐出格式完全合法的 `nav_msgs::msg::Path`，供系统在 RViz 中渲染和让控制器跟踪。
   - **未来替换方案**：
     - 学完移动机器人路径规划后，继承该接口实现 `AStarPlanner` 或挂接 Nav2 规划插件，无需改动调度器核心。

2. **`IPathTracker`（路径跟踪接口）**：
   - 接口定义：`virtual Twist2D ComputeControl(const Pose2D& current_pose, const std::vector<Pose2D>& path, double lookahead_dist) = 0;`
   - **当前占位实现 (`PurePursuitTracker` / `StanleyTracker`)**：
     - 当前已实现的 Pure Pursuit 和 Stanley 已经足够轻量（纯几何前视距离投影，几十行 C++ 代码）。保留其作为默认基准追踪器。
   - **未来替换方案**：
     - 预留未来拓展模型预测控制（MPC）或二次型调节器（LQR）的参数结构体与派生类。

---

## 三、 从 robot-project 借鉴与落地的核心系统机制

### 1. 可观测性：RViz2 3D 拓扑路网与死锁渲染节点
- **功能目标**：将抽象的资源锁和任务状态以三维图形化形式展现。
- **新增节点**：`dispatcher_visualizer_node`（位于 `amr_dispatcher_ros`）。
- **具体实现**：
  - **路网拓扑展示**：订阅拓扑配置，以 `Marker::LINE_LIST` 绘制道路网络（灰色/蓝色）。
  - **路段占用（Lock）可视化**：当某辆 AMR 申请到路段 A->B 的原子锁后，该路段以动态粗线条变为**高亮绿色**，并在路段上方绘制持有该锁的小车 ID。
  - **死锁预警（Deadlock Warning）**：死锁巡检线程一旦在依赖图中检测到环（Cycle），在死锁发生的节点/路口正上方绘制红色半透明警戒圆柱 `Marker::CYLINDER`，并悬浮 3D 警报文字 `[DEADLOCK DETECTED: Car1 <-> Car2]`。
  - **解除恢复动画**：当调度系统强制让低优先级车辆释放锁并后退后，警戒圆柱消失，通行路线恢复为绿色。

### 2. 控制面：单文件 Web 运营看板与 REST 桥接增强
- **功能目标**：为整个系统提供开箱即用的工业运维看板，脱离黑盒调试。
- **搬运与改造**：
  - 从 `robot-project/tools/operator_console.html` 搬运并适配为 `amr_dispatcher/tools/operator_console.html`。
  - **强化 `rest_gateway.cpp`**：
    - 新增 GET `/api/operator/snapshot`：聚合输出完整的系统状态 JSON（含：任务积压总数、活跃任务列表、当前生效调度算法、死锁标志、各路段锁持有者列表、硬件链路健康度）。
    - 强化 POST `/api/operator/submit_order`：支持通过前端直接点选起点站点、终点站点、优先级、截止时间，带幂等性校验。
    - 强化 POST `/api/operator/estop`：支持从 Web 页面一键触发全系统软急停与恢复。
  - **免配置访问**：可以直接在本地通过浏览器打开该 HTML，或者由 `rest_gateway` 增加轻量级静态文件路由（访问 `http://localhost:8080` 直接返回该看板）。

### 3. 安全机制：多源安全仲裁安全门与看门狗强化
- **借鉴点**：`robot-project` 中 `cmd_vel` 永远经过独立仲裁门的思想。
- **具体实现**：
  - 完善 `safety_gate_node.cpp`：将其作为速度下发的唯一出口（所有其他节点只能发往 `/tracking_cmd_vel` 或 `/teleop_cmd_vel`，只有 Safety Gate 能发往底盘驱动的 `/cmd_vel`）。
  - **多源仲裁优先级**（由高到低）：
    1. **硬/软 E-Stop（急停）**：立即下发零速，锁死输出；
    2. **Software Watchdog（软看门狗超时）**：若 `dispatcher` 或 `path_tracker` 心跳丢失超过 200ms，立即触发故障保护刹车；
    3. **降级限速（Degraded Speed Limit）**：若底层硬件驱动上报链路丢包率较高，动态限制最大输出速度至 0.2 m/s；
    4. **正常通过（Normal Pass-through）**：平滑输出控制指令。

### 4. 资源调度扩展：物理设施（充电桩/自动门）抽象锁
- **借鉴点**：`robot-project` 的 `facility_reservation.hpp`。
- **业务价值**：将拓扑锁推广为“通用资源调度器”，不再局限于路段，使系统更贴近实际工厂。
- **纯系统机制实现**：
  - 在 `amr_dispatcher_core` 的资源锁管理器中，新增 `FacilityResource` 类型：
    - **自动门（Door）**：支持申请、开启确认、通行占用、释放超时关闭。
    - **充电桩（ChargingStation）**：支持电量低强制抢占、预约排队、充电锁死。
  - 完全由状态机驱动，支持基于时间戳的租约过期自动释放（Lease Mechanism），彻底避免物理设施因单车失联导致全局永久被锁。

### 5. 质量保障：场景化混沌演练与自动化验收脚本
- **借鉴点**：`robot-project` 的 110+ Shell 测试脚本工程体系。
- **落地脚本矩阵**（放置于 `scripts/`）：
  - `check_deadlock_resolution.sh`：自动模拟并发提交 2 个冲突对头订单，检查系统是否触发死锁警报，并验证抢占释放机制是否在预期时间内解除冲突。
  - `check_hardware_failover.sh`：向通信链路注入人为丢包，验证系统在无需人工干预的情况下，按顺序自动降级：`Serial -> UDP -> Mock`。
  - `check_watchdog_estop.sh`：通过 Linux 信号临时冻结控制节点进程（`kill -STOP`），验证 Watchdog 是否在 200ms 内准确触发并发布停止指令。
  - `check_all.sh`：一键自动化全流程流水线，串联静态检查、单元测试、基准压测与上述混沌测试。

---

## 四、 ROS 2 中间件技术栈深度落地规划

### 1. 节点生命周期（Lifecycle）全面改造
- **目标**：主节点（如 `dispatcher_node`、`chassis_driver_node`）全部继承自 `rclcpp_lifecycle::LifecycleNode`。
- **状态转移实现**：
  - `on_configure()`：解析配置文件、初始化锁拓扑与内存结构、创建各类通信实体，保持未激活（Inactive）。
  - `on_activate()`：激活所有的 `LifecyclePublisher`，启动看门狗定时器，正式允许接收外部调度请求。
  - `on_deactivate()`：停止接收任务，下发底盘驻车指令，释放暂存的未锁定资源。
  - `on_cleanup()` / `on_shutdown()`：断开网络/串口硬件连接，清空内部队列。
- **运维收益**：支持标准的 `ros2 lifecycle get/set` 运维纳管。

### 2. 任务调度 Action 体系与抢占机制
- **目标**：使用工业标准的 ROS 2 Action 替代脆弱的单向 Topic 或阻塞式 Service。
- **接口定义**：`amr_dispatcher_interfaces/action/ExecuteMission.action`
  ```text
  # Goal
  string mission_id
  uint8 priority
  string target_station
  float64 deadline_seconds
  ---
  # Result
  bool success
  string message
  float64 execution_time
  ---
  # Feedback
  uint8 status             # QUEUED, WAITING_LOCK, EXECUTING, DEGRADED
  string current_node
  float32 percent_complete
  float64 remaining_eta
  ```
- **关键机制**：
  - **任务取消与抢占（Preemption）**：支持高优先级任务到来时，调用当前运行中任务的 `handle_cancel()`，节点优雅下发刹车、原子释放已持有的边锁，转入下一个任务执行。

### 3. 工业级 QoS 策略与诊断系统接入
- **差异化 QoS 配置**：
  - 传感器与里程计数据（`/odom`）：配置为 `rclcpp::SensorDataQoS()`（Best Effort、小深度缓冲区），避免通信延迟累积。
  - 调度状态、拓扑锁事件、急停指令：配置为 `Reliable` + `TransientLocal`（保持历史最后一条），确保后启动的 Web 网关或 RViz 节点上线后立即同步到最新系统状态。
- **接入 `diagnostic_updater`**：
  - 周期性向 `/diagnostics` 广播整机健康报告。
  - 监控项包括：看门狗各源心跳延迟、底盘通信丢包率、当前调度队列积压长度、死锁报警标志位。
  - 支持通过标准的 ROS 2 图形化监控工具 `rqt_runtime_monitor` 查看。

### 4. 动态参数在线热重载（Dynamic Parameters）
- **实现机制**：通过 ROS 2 的 `add_on_set_parameters_callback`。
- **支持热更项**：
  - 调度策略算法切换：`scheduling_strategy`（可在 `"PriorityFIFO"`、`"EDF"`、`"SJF"`、`"Score"` 间无感平滑切换）。
  - 死锁超时阈值：`deadlock_timeout_ms`。
  - 安全门限速值：`max_linear_vel`、`degraded_linear_vel`。
- **体验效果**：执行 `ros2 param set /dispatcher_node scheduling_strategy "EDF"`，无需重启进程，系统下一轮调度即刻切换算法。

---

## 五、 分阶段实施里程碑与时间表

```
┌────────────────────────────────────────────────────────────────────────┐
│  Phase 1: 可视化与人机交互赋能（最快出展示效果，1~2天）                       │
│  • 搬运并适配 operator_console.html Web 控制台与 rest_gateway 端点对齐   │
│  • 开发 dispatcher_visualizer_node，实现 RViz2 拓扑路网、锁与死锁 3D 渲染 │
└───────────────────────────────────┬────────────────────────────────────┘
                                    │
                                    ▼
┌────────────────────────────────────────────────────────────────────────┐
│  Phase 2: ROS 2 中间件规范化改造（深度体现中间件工程，2~3天）               │
│  • 实现 ExecuteMission.action 动作服务与任务异步流式反馈/取消抢占机制     │
│  • dispatcher_node 升级为标准 rclcpp_lifecycle::LifecycleNode          │
│  • 参数动态声明与在线热更回调（无缝切换调度算法与限速）                    │
│  • 统一 QoS 划分与 diagnostic_updater 状态自检上报                      │
└───────────────────────────────────┬────────────────────────────────────┘
                                    │
                                    ▼
┌────────────────────────────────────────────────────────────────────────┐
│  Phase 3: 架构接口标准化与设施抽象（完成占位，2天）                          │
│  • 建立 IPathPlanner 与 IPathTracker 纯虚接口与 StraightLineMock 实现    │
│  • 抽象 FacilityResource（充电桩/自动门租约锁与超时保护机制）              │
│  • 强化 safety_gate_node 多源仲裁与 Watchdog 联动闭环                    │
└───────────────────────────────────┬────────────────────────────────────┘
                                    │
                                    ▼
┌────────────────────────────────────────────────────────────────────────┐
│  Phase 4: 混沌工程与自动化交付体系（答辩/面试终极武器，1~2天）               │
│  • 编写 check_deadlock_resolution.sh（死锁检测与恢复演练）               │
│  • 编写 check_hardware_failover.sh（通信断开三级自动降级演练）             │
│  • 编写 check_watchdog_estop.sh（进程冻结看门狗急停保护演练）              │
│  • 编写一键执行汇总脚本 check_all.sh                                    │
└────────────────────────────────────────────────────────────────────────┘
```

---

## 六、 答辩与面试展示全景剧本（5 分钟演示流）

当你向老师、考官或大厂技术面试官展示本项目时，可直接按照以下节奏进行：

1. **【0:00 - 0:45】全景展示与架构介绍**：
   - 启动 Docker 系统，浏览器打开 `operator_console.html`，同时打开 RViz2。
   - 介绍核心理念：“这是一个将领域算法与 ROS 2 中间件严格解耦的高可用调度平台，核心算法库零 ROS 依赖，预留了标准化算法占位，外层则将 ROS 2 的中间件能力发挥到极致。”
2. **【0:45 - 2:00】Web 控制台提单与 RViz 3D 拓扑锁演示**：
   - 在 Web 页面点选下发一个从 Station A 到 Station B 的运输订单。
   - 在 RViz2 中展示被小车锁定的道路立刻**变为高亮绿色**，小车平稳行进。
   - 随后下发一个对向冲突的高优先级订单，展示系统如何识别资源争抢，将低优先级小车停在路口，并在 RViz 中弹出**红色死锁警戒圆柱**与警告标签。
   - 演示死锁仲裁器介入，低优先级小车退让让行，死锁自动解除并恢复绿灯通行。
3. **【2:00 - 3:15】混沌工程演练：看门狗与高可用三级降级**：
   - 在终端运行 `./scripts/check_watchdog_estop.sh`，人为冻结控制节点。
   - 观察 Safety Gate 在 200ms 内准确触发软看门狗超时，控制指令全部清零制动，防止实车失控飞车。
   - 模拟串口通信中断，展示底层链路质量监控如何驱动通信方式从 `Serial` 无缝退化为 `UDP` 并继续运行。
4. **【3:15 - 4:00】中间件高级特性：在线热更与生命周期管理**：
   - 在终端执行命令：`ros2 param set /dispatcher_node scheduling_strategy "EDF"`。
   - 打开 Web 页面或观察日志，展示调度核心在不重启节点的情况下，无感热更算法。
   - 展示标准 `ros2 lifecycle` 状态机受控管理。
5. **【4:00 - 5:00】底层功底与性能数据收尾**：
   - 切入终端运行 `./scripts/build.sh`，展示纯 CMake 下 **130+ 单元测试在 4 秒内全绿通过**。
   - 运行 `./scripts/benchmark.sh`，展示 **2,000,000 条/秒的无锁异步日志吞吐** 与 **25,000 ops/秒的任务队列并发吞吐**。
   - 总结：“这套系统不仅做到了极致的软件工程整洁度，还通过良好的设计模式为团队后续接入更前沿的 MPC 算法或全局路径规划算法打好了坚实的基石。”
