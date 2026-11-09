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
- [x] 异步低阻塞日志器与同步双实现
- [x] ROS 2 Lifecycle 调度节点、底盘驱动节点、安全门节点、循迹节点
- [x] 基于 hardware_interface::SystemInterface 的 ros2_control 硬件插件
- [x] 交互式 CLI 诊断客户端与 RESTful HTTP 提单网关（带幂等性校验）

## Phase 2: 工业级与高可靠增强 (Industrial Reliability - 2026 Q4)
- [x] **全局地图拓扑规划器整合 (Complete)**：
  - 支持官方 ROS 2 `nav2_map_server` 格式地图包 (yaml/pgm) 导入与解析
  - 纯 C++ 核心安全空间圆形核腐蚀、未知区封锁、Zhang-Suen 细化骨架提取与受保护毛刺剪枝
  - 自动提取带真实 `polyline` 几何折线的路网，站点正交投影拆边接入拓扑
  - `SkeletonPlanner` 真实折线 0.1m 等距插值规划器，彻底解决走廊拐弯直线穿墙隐患
  - 全路线原子排他锁与 `map_hash` 强一致性版本门禁
- [x] **VDA 5050 协议原生对接 (Complete)**：
  - 支持 VDA 5050 v2.0+ 标准 JSON MQTT 订单与状态报文
  - 将 VDA 5050 `order` 映射为 `MissionQueue` 中的任务节点与时空轨迹
  - dispatcher_node 直发 `/fleet/{vid}/send_order`（orderId=mission_id），
    gateway 网关双向桥接 MQTT；Task 5 逐段路权动态释放 (Spec 5.3) +
    断链锁保留与调度挂起守卫 (Spec 5.2)；
    操作员强清 `/dispatcher/force_clear_vda5050_link` 闭合 Spec 5.2 人工强解半边
    （摘链 + 释锁 + 收口在途任务 + 解除挂起；在线车拒绝误杀）；
    端到端验收 `./scripts/check_vda5050_e2e.sh`（8 阶段全 PASS）
- [ ] **分布式时空窗交通协调 (Spatio-Temporal Reservation)**：
  - 从当前的静态资源原子锁升级为带时区的时间窗口预约（Time-expanded Reservation Graph）
  - 允许多机器人在不冲突的时间段共享同一交叉路口或通道

## Phase 3: 多机集群与智能化协同 (Fleet Coordination & AI - 2027 Q1)
- [ ] **分布式对等协商 (Peer-to-Peer Conflict Resolution)**：
  - 当中心调度离线或网络分区时，机器人之间通过局域网广播进行右侧让行或优先级协商
- [ ] **自适应速度平滑曲线**：
  - 循迹控制器集成 S 型加减速（Jerks/Acc 约束）及根据路面曲率的动态前瞻降速
- [ ] **能耗感知调度策略 (Battery-Aware Fleet Dispatching)**：
  - 引入 `EnergyOptimalComparator`，结合站点几何距离与电池剩余电量自动插入充电机动任务
