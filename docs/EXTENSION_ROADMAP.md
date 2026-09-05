# AMR Dispatcher 演进路线图 (Extension Roadmap)

## Phase 1: 当前已落地 (Core Baseline & ROS 2 Adaptation - Complete)
- [x] 纯 C++17/20 核心库解耦，脱离 ROS 独立编译构建与 120 个单测验证
- [x] Mick 二进制与文本双流协议编解码、粘包/半包流式解析
- [x] Pure Pursuit 与 Stanley 循迹控制器纯类化
- [x] 可插拔调度比较器策略模式（PriorityFIFO / EDF / SJF / WeightedScore）
- [x] 交通管制拓扑原子锁（边与节点互斥表）
- [x] 任务死锁检测（长持资源/无进展）与自愈恢复动作决策
- [x] 硬件通信多级降级控制器（Serial -> UDP -> Mock）与指数退避重连
- [x] 多源安全仲裁门控（CmdVelGate）与节点心跳软看门狗
- [x] 强类型 YAML 配置加载器、Schema 校验与文件变动热重载
- [x] 异步无锁日志器与同步双实现
- [x] ROS 2 Lifecycle 调度节点、底盘驱动节点、安全门节点、循迹节点
- [x] 基于 hardware_interface::SystemInterface 的 ros2_control 硬件插件
- [x] 交互式 CLI 诊断客户端与 RESTful HTTP 提单网关（带幂等性校验）

## Phase 2: 工业级与高可靠增强 (Industrial Reliability - 2026 Q4)
- [ ] **VDA 5050 协议原生对接**：
  - 支持 VDA 5050 v2.0+ 标准 JSON MQTT 订单与状态报文
  - 将 VDA 5050 `order` 映射为 `MissionQueue` 中的任务节点与时空轨迹
- [ ] **分布式时空窗交通协调 (Spatio-Temporal Reservation)**：
  - 从当前的静态资源原子锁升级为带时区的时间窗口预约（Time-expanded Reservation Graph）
  - 允许多机器人在不冲突的时间段共享同一交叉路口或通道
- [ ] **全局地图拓扑规划器整合**：
  - 将 `station_catalog` 的 Dijkstra / A* 最短拓扑路径搜索封装为轻量独立模块
  - 路径生成器与 `PathTrackerNode` 形成闭环航线重规划

## Phase 3: 多机集群与智能化协同 (Fleet Coordination & AI - 2027 Q1)
- [ ] **分布式对等协商 (Peer-to-Peer Conflict Resolution)**：
  - 当中心调度离线或网络分区时，机器人之间通过局域网广播进行右侧让行或优先级协商
- [ ] **自适应速度平滑曲线**：
  - 循迹控制器集成 S 型加减速（Jerks/Acc 约束）及根据路面曲率的动态前瞻降速
- [ ] **能耗感知调度策略 (Battery-Aware Fleet Dispatching)**：
  - 引入 `EnergyOptimalComparator`，结合站点几何距离与电池剩余电量自动插入充电机动任务
