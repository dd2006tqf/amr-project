# AMR Dispatcher 关键技术决策记录 (Architecture Decision Records)

## ADR-001: 核心算法与 ROS 2 完全解耦

### 背景
原仓库 `robot-project` 中，Pure Pursuit、Stanley、调度队列、死锁检测均以内联方式实现在 ROS 节点内部。导致的问题：
1. 任何单测必须启动完整 ROS 2 守护进程、DDS 中间件与参数服务器。
2. 单元测试执行慢（数秒至数十秒），CI 构建时间长，难以融入轻量级 CI 流水线。
3. 算法核心逻辑与通信框架耦合，无法跨平台移植至裸机或嵌入式 Linux 控制器。

### 决策
在 `amr_dispatcher_core` 中完全禁用 ROS 依赖，所有输入输出均为纯 C++ 基础类型或 STL 结构：
- `PurePursuitController`：输入 `Pose2D` + `std::vector<Waypoint2D>`，输出 `PurePursuitResult`。
- `StanleyController`：输入 `Pose2D` + `speed` + `waypoints`，输出 `StanleyResult`。
- `MissionQueue`：输入 `Mission`，依托 `MissionComparator` 纯虚接口排序。

### 结果
- 核心库编译时间 < 3 秒，120 个单测在 4.6 秒内执行完毕。
- ROS 2 节点退化为纯 Adapter，只负责序列化与反序列化，极大降低了系统调试难度。

---

## ADR-002: 调度排序重构为策略模式 (Strategy Pattern)

### 背景
原项目使用硬编码的 `priority` 整数排序，若需要支持截止期最早优先（EDF）或最短作业优先（SJF），需要复制整个任务队列并在多处侵入式修改代码。

### 决策
抽象基类 `MissionComparator`，通过重写严格弱序函数 `Before(const Mission& a, const Mission& b)` 注入不同的排序规则：
1. `PriorityFifoComparator`：优先级优先，同优先级依 `sequence` 先到先服务。
2. `DeadlineComparator`：按 `deadline_unix_ms` 绝对时间戳升序排序。
3. `ShortestJobComparator`：按预估执行时间 `expected_seconds` 升序排序。
4. `WeightedScoreComparator`：加权评分 $S = w_p \cdot P + w_d \cdot \text{Score}_d + w_s \cdot \text{Score}_s$。

### 结果
新增任意调度策略仅需添加一个类并向工厂注册一行代码，符合开闭原则（OCP）。

---

## ADR-003: 环形缓冲区 (Ring Buffer) 替代动态容器存储审计事件

### 背景
`MissionEventLog` 需要记录高频的状态机跃迁与调度事件供诊断和回放。若使用 `std::vector`，长期运行会导致内存单调增长或频繁的 `erase(begin())` 引起 $O(N)$ 内存移动。

### 决策
采用固定容量（Capacity）的环形缓冲区，底层使用预分配的固定数组，维护 `head_` 索引与当前 `size_`。当缓冲区满时，新事件原地覆盖最旧槽位。

### 结果
- 追加事件的时间复杂度严格保证为 $O(1)$，内存消耗恒定，杜绝运行时内存碎片。
- 支持指定数量的最新事件倒序提取（Recent）。

---

## ADR-004: 多级硬件通信降级与指数退避重连

### 背景
底盘通信极易受工业现场电磁干扰（EMI）、线缆松脱或无线局域网抖动影响。原项目在串口断开后直接崩溃或静默阻塞。

### 决策
1. **滑动窗口质量监控**：记录最近 64 帧的丢包率、CRC 校验错及往返时延（RTT）。
2. **多级硬件降级**：预置 `Serial` -> `UDP` -> `Mock` 优先级列表，连续丢包超过 30% 且超出冷却时间（30s）后，自动切换至下一级通道，防止小车失控。
3. **退避自愈**：在底层通道断开时，采用指数退避算法（200ms 起步，上限 10s）并叠加 20% 随机抖动，避免总线重连雪崩效应。
