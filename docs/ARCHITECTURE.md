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
|  - CmdVelGate    - ConfigLoader    - AsyncLogger (异步低阻塞队列)|
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

---

## 5. 可观测性与运行时拓扑（相位 11）

### 5.1 探针分层：liveness / readiness / 组件语义

三者回答的是**不同**的问题，混在一起就会得出错误结论：

| 层 | 端点 | 只看什么 | 它**不**看什么 |
|---|---|---|---|
| liveness | `GET /healthz` | 网关进程还能处理 HTTP | 任何 ROS 状态（DDS 全挂时仍 200，这是定义） |
| readiness | `GET /readyz` | 四个组件是否"新鲜且语义健康" | 业务数据 |
| 组件语义 | 同上 `components[].reason` | 每个组件具体坏在哪 | — |

**快车道**：`/healthz` 与 `/readyz` 由 `accept` 线程读完请求头后**同步**应答，
不进工作线程队列。这不是优化而是正确性要求：4 个 worker 每个都可能
`wait_for_service(1s)`，DDS 卡死时它们会被全部占住，探针将和它要检测的
故障同时失效。反过来，accept 是单点，所以它读头有 **50ms 预算**
（`kAcceptReadBudgetMs`）——超过就立刻入队交给 worker 去等，主线程马上回到 accept。

### 5.2 自愈链：谁负责拉起、谁负责提拉

```
进程崩溃 ──► launch respawn 起新进程 ──► 新进程 unconfigured
                                              │
                                              ▼
                         lifecycle_coordinator_node (auto_bringup, 500ms)
                                              │
                                              ▼
                                  configure() ──► activate()
                                              │
                                              ▼
                        on_configure 里注册的服务重新出现（网关不再 503）
```

三点必须记住：

1. **respawn 只对"进程死了"起作用**，不做健康判定。一个活着但卡死的节点不会被重生。
2. **重生后的提拉不是 launch 做的。** launch 的 `EmitEvent(CONFIGURE)` 与
   `OnStateTransition(inactive→ACTIVATE)` 只在首次启动触发一次。
   所以协调器死亡 = 自愈链断裂（协调器自己也在 respawn 名单里）。
3. **冷重启会重派在途任务**：`on_configure` 从 `state_snapshot_file` 恢复队列并把
   在途任务重新入队，车辆可能收到第二份订单。这是既有语义，不是相位 11 引入的。

这条链由 `test/l2_launch_contracts/test_supervision_contracts.py` 在**不依赖 Docker**
的前提下钉死（域 94）：kill -9 → 新 PID → 协调器重新提拉 → `on_configure`
的服务重新可调用。

### 5.3 告警状态机：为什么"多久发一次"是契约不是细节

```
Observe(sample) ──► 连续 N 次不就绪? ──是──► firing（受 min_interval 节流）
                        │否                        │
                        ▼                          ▼
                     不动作              恢复 -> 恰好一条 *_recovered（不受节流）
```

- 连续 N 次才发：滤掉单拍抖动（DDS 重连、一次 GC 都会造成一拍延迟）。
- 恢复通知**不受节流**且**每次故障恰好一条**：漏发恢复会让人以为事故还在；
  重复发恢复会让人以为它来回抖动。
- 发送在**独立线程**（唯一做阻塞 IO 的地方），executor 只负责组装与入队。
- 队列满丢**最旧**：一条 5 分钟前的 "not ready" 不如刚刚这条有用。

这三条语义（`test_alerting.cpp`）与现场行为（`check_health_supervision_e2e.sh` T5
的状态机不变量断言）双重锁定。T5 断言的不是"收到几条"，而是
"每条 recovered 之前都有一条未被配对的 firing、没有连续两条 recovered、
最终 resolved" —— 因为 respawn 期间真实存在 flapping，数条数没有意义。

### 5.4 运行时拓扑叠加层：为什么它不是"新地图版本"

```
stations.yaml (base)  ──┐
                        ├──► ApplyTopologyOverlay ──► EffectiveCatalog()
overlay.yaml (overlay) ─┘         （就地改 enabled/bidirectional）
                                                    │
                          ┌─────────────────────────┼─────────────────────────┐
                          ▼                         ▼                         ▼
                   FindRoutePath            ReserveLocks             EstimateDistance
                   SkeletonPlanner          CBS / TimeSpace          ListStations
```

关键决策：**封路 = `enabled=false`，单向 = `bidirectional=false`**（就地改目录副本），
而不是维护一份"禁用键"运行态。理由：`FindRoutePathImpl` 已经同时尊重这两个字段，
而 `FindRoutePathAvoiding` 的键是**无序**的，用它表达单向会连合法方向一起封死。
这样做之后，五个下游消费者（规划、折线展开、距离估算、时空窗、CBS）**自动一致**，
不需要改任何一个调用点。

`map_hash` 原样拷贝，所以叠加层**不是**新地图版本：状态快照的版本门禁
（拒绝把昨天的队列套到今天的地图上）完全不受影响。这也是"施工区能被热生效"
而"地图换版必须重启"这两件事能共存的原因——判据不同。

**在途仲裁**：`lastNodeId` 只说明最后确认到达的节点，车辆可能正驶在下一段上。
因此"剩余航段的第一段被封" = 可能正压在上面 → 默认 409。

**写前持久化**：落盘是提交管线的**第一步**。进程在落盘之后任意时刻死掉，
重启回放就是目标态；死在落盘之前，哪里都没变。这免掉了"磁盘失败要回滚内存锁"
这个最难写对的分支——代价是一个微秒级的"文件新、内存旧"窗口，在单线程
`rclcpp::spin` 下不可观测。
