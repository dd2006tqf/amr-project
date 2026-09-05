# AMR Dispatcher 详细系统架构设计 (Architecture Specification)

## 1. 整体架构与分层设计原则

`amr_dispatcher` 采用了严格的**分层解耦架构**。整个系统自底向上划分为四层：

```
+-------------------------------------------------------------+
|               4. 交互与运维工具层 (Tools)                     |
|     - dispatcher_cli (交互终端)    - rest_gateway (REST API) |
+-------------------------------------------------------------+
                              ▲
                              │ ROS 2 话题 / 服务 / Action
                              ▼
+-------------------------------------------------------------+
|               3. ROS 2 运行时封装层 (ROS Nodes)               |
|  - chassis_driver_node         - dispatcher_node (Lifecycle)|
|  - safety_gate_node            - path_tracker_node          |
|  - link_health_publisher       - chassis_hardware_interface |
+-------------------------------------------------------------+
                              ▲
                              │ 纯 C++ 头文件与对象调用 (零 ROS 依赖)
                              ▼
+-------------------------------------------------------------+
|               2. 核心算法与业务逻辑层 (Core Library)           |
|  [chassis]       [path_tracking]   [dispatcher]             |
|  - PacketStream  - PurePursuit     - MissionQueue (策略模式)|
|  - Kinematics    - Stanley         - TrafficReservationTable|
|  - LinkQuality   - PathGeometry    - DeadlockDetector       |
|  - LinkRecovery                    - RecoveryPolicy         |
|  - Degradation                     - MissionEventLog (环形缓冲)
|  [safety]        [config]          [logging]                |
|  - CmdVelGate    - ConfigLoader    - AsyncLogger (无锁队列) |
|  - Watchdog      - HotReload       - SyncLogger             |
+-------------------------------------------------------------+
                              ▲
                              │ 硬件与操作系统底层抽象
                              ▼
+-------------------------------------------------------------+
|               1. 基础设施与操作系统抽象 (OS & Hardware)        |
|  - POSIX Serial TTY (/dev/ttyUSB*) - UDP Socket (BSD)       |
|  - Linux Steady Clock / High-res Timers - Standard Threads  |
+-------------------------------------------------------------+
```

### 1.1 核心解耦设计原则 (Core-First Philosophy)
- **Zero ROS Dependency in Core**: `amr_dispatcher_core` 目录下的所有源文件严禁包含任何 `rclcpp`、`std_msgs` 或 ROS 头文件。使用标准 C++17/20 STL、POSIX 套接字和 `yaml-cpp` 实现。
- **High Testability**: 核心库脱离 ROS 运行时环境，可在纯 Linux 主机上使用 CMake + GoogleTest 进行快速单元测试与极速 CI/CD 验证。
- **Adapter-based Node Layer**: ROS 2 节点仅充当外壳适配器（Adapter），负责将 ROS 话题/服务转换为 C++ 数据结构喂给 Core，并获取返回值发布出去。

---

## 2. 调度器状态机与迁移规范 (Dispatcher Lifecycle & State Machine)

任务生命周期采用**4 态设计 + 终态结果分离**模型：

```
             +--------------------+
             |      PENDING       |<------------------+
             +--------------------+                   |
               |                ▲                     |
               | Dispatch       | Pause/Preempt       |
               v                |                     |
             +--------------------+                   |
             |       ACTIVE       |                   |
             +--------------------+                   |
               |                ▲                     |
               | Pause          | Resume              |
               v                |                     |
             +--------------------+                   |
             |       PAUSED       |-------------------+
             +--------------------+
               |
               | Success / Failure / Cancel
               v
             +--------------------+
             |      FINISHED      |
             +--------------------+
             (Result: Succeeded / Failed / Canceled)
```

### 2.1 状态转移合法性校验矩阵

| 原状态 (From) | 目标状态 (To) | 合法性 | 触发时机 / 约束说明 |
|---|---|---|---|
| `kPending` | `kActive` | ✅ 允许 | 调度器根据所选策略出队分发至小车 |
| `kActive` | `kPaused` | ✅ 允许 | 避障长时受阻、人工介入、死锁策略介入暂缓 |
| `kActive` | `kFinished` | ✅ 允许 | 导航完成、失败或被显式取消（必须附带非空 Result） |
| `kPaused` | `kActive` | ✅ 允许 | 恢复继续执行当前任务 |
| `kPaused` | `kPending` | ✅ 允许 | 退回等待队列重新参与下一轮策略打分 |
| `kPaused` | `kFinished` | ✅ 允许 | 在暂停状态下直接终止任务 |
| `kFinished` | 任何状态 | ❌ 禁止 | 终态不可逆，禁止二次跃迁 |

---

## 3. 硬件链路监控、自愈与自动降级机制 (Link Quality & Degradation)

```
   +-------------------------------------------------------------+
   |                  ChassisDriverNode 数据流                   |
   +-------------------------------------------------------------+
               |                                     ▲
        /cmd_vel_safe                         /odom, /link_health
               v                                     |
   +-------------------------------------------------------------+
   |            BackendDegradationController (调度控制器)          |
   |                                                             |
   |   Preferred List:                                           |
   |   [0] SerialBackend  <--- 当前使用                           |
   |   [1] UdpBackend     <--- 降级备用                           |
   |   [2] MockBackend    <--- 仿真/安全兜底                     |
   +-------------------------------------------------------------+
         |                                           ▲
      写/读采样                                    质量度量
         v                                           |
   +-------------------------------------------------------------+
   |               LinkQualityMonitor (滑动窗口监控)              |
   |   - 丢包率 = (CRC错 + 丢包) / 总帧数                         |
   |   - RTT 往返时延平均值与最大值                               |
   |   - 健康评级: HEALTHY (≤2%) / DEGRADED (≥15%) / CRITICAL     |
   +-------------------------------------------------------------+
         |                                           ▲
      连续超时/严重丢包                             重试恢复
         v                                           |
   +-------------------------------------------------------------+
   |               LinkRecoveryPolicy (自愈退避状态机)             |
   |   - 指数退避: backoff = min(initial * 2^(n-1), max)          |
   |   - 随机抖动 (Jitter Ratio: 0.2) 避免总线重连雪崩            |
   +-------------------------------------------------------------+
```

---

## 4. 安全多源门控与仲裁机制 (Safety Multi-Source Arbitration)

`safety_gate_node` 聚合来自急停、碰撞保险杠、看门狗心跳、超速监控的信号，输出经过安全裁决的控制指令：

```
/cmd_vel_raw ────┐
/safety/estop ───┼──► [CmdVelGate 多源仲裁门] ──► /cmd_vel_safe
/safety/bumper ──┤     │
Watchdog 超时 ───┘     ├── 严重事件 (E-Stop / Bumper / Soft Timeout) ──► 立即全停 (速度清零, motion_allowed=false)
                       └── 预警事件 (低电量 / 心跳延迟 / 缓冲区满)   ──► 动态限速 (缩放为原速度 30%)
```
