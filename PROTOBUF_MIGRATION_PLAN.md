# Protobuf 接入 ROS 2 话题：完整替换方案

> 状态：阶段 0（包骨架 + 技术验证）已完成并**实测通过**。
> 本文档只记录已验证的事实、真实的迁移清单、分阶段步骤，以及不可回避的边界。
> 日期：2026-11-05

---

## 0. 结论先行

用**零 fork 的 typesupport 注入**，让 protobuf 消息直接作为 ROS 2 话题类型使用，
在 ROS 2 Jazzy 上已实测可行：

- C++ 侧 `create_publisher<ProtoT>` / `create_subscription<ProtoT>` 正常工作
- `ros2 amr echo` 能解析出完整字段（含嵌套 message、enum、uint64 大数值）
- 端到端 249 帧零丢帧，字段逐项校验通过

代价是三条硬约束，必须先接受：

1. **只覆盖话题（topic）**。服务与动作明确不支持，见 §5.1。
2. **`ros2 topic echo` / RViz / rosbag 回放不认**。已提供替代工具 `ros2 amr echo`，见 §5.2。
3. **DDS 类型名变了**，与既有 rosidl 话题**不能直接互通**，只能双发过渡或全量切换，见 §4。

---

## 1. 动机

现有 7 个自定义消息（`amr_dispatcher_interfaces/msg/`）由 rosidl 生成，采用**固定宽度**编码：
`uint64` 恒占 8 字节，`bool` 恒占 1 字节。而 `frames_total`、`commands_sent` 这类计数器
数值单调递增、高位长期为 0，固定宽度编码浪费明显。

但省字节不是唯一动机，甚至不是主要动机：

| 维度 | rosidl | protobuf |
|---|---|---|
| 跨语言 | Python/C++ 有生成器；Rust/Go/Java 需另找 | 官方支持十几种语言 |
| 向前兼容 | 加字段要重新生成 + 重新编译 + 重启 | 字段编号 + `reserved`，老端忽略新字段 |
| 枚举 | 只能靠字符串约定（现状 `tier` 就是 string） | 编译期检查 |
| 工具生态 | 绑定 ROS | `protoc`、gRPC、buf、schema 管理 |

需要说明的是：**本项目当前没有跨语言需求，也没有热升级需求**。
引入 protobuf 的收益主要是「未来可能性」与「编码效率」，属于工程投资而非痛点解决。
如果团队更看重「所有 ROS 工具开箱即用」，那么留在 rosidl 是完全合理的选择（见 §7）。

---

## 2. 技术方案

### 2.1 原理：两个特化点

ROS 2 在 C++ 侧用函数指针表 `rosidl_message_type_support_t` 描述一个消息类型。
`rclcpp::Publisher<T>` 通过 **ADL 找到的模板特化点** 获取它：

```cpp
rosidl_typesupport_cpp::get_message_type_support_handle<T>()
```

**我们为 protobuf 类型提供自己的特化**，从而绕过整个 rosidl 代码生成链路。
无需修改 rclcpp / rcl / rmw / rosidl 的任何一行源码。

还必须同时提供另一个特化点，否则 rclcpp 内部会走 rosidl 分支：

```cpp
rosidl_generator_traits::is_message<T>   // 必须返回 true
```

### 2.2 Jazzy 的四个强制回调——本方案与 Humble 的分水岭

Jazzy 的 `rcl` 会在以下位置**无条件解引用**函数指针。源码位置已逐条核对：

| 回调 | 调用点 | 缺失后果 |
|---|---|---|
| `get_type_hash_func` | `rcl/publisher.c:133` | SEGFAULT（跳转 0x0） |
| `get_type_description_func` | 同上 | SEGFAULT |
| `get_type_description_sources_func` | 同上 | SEGFAULT |
| introspection（经 `ts.func` 分发） | `TypeSupport_impl.cpp:612` | `failed to register type object with incompatible type` |

**这四个在 Humble 上都不存在。** 这是为什么把上游 ROS2-Protobuf 补丁
「改名搬过来」行不通——不是重命名，而是多了一层必须实现的协议。

各回调的实现方式：

- **type hash**：对 `FileDescriptorProto` 序列化后取 SHA-256，版本字节 `0x01`（RIHS01）。
- **type description**：返回合成的 `TypeDescription`，只填名称，字段表为空。
- **type description sources**：`encoding = "dynamic"`（官方定义值），`raw_file_contents` 填 `.proto` 源文本。
- **introspection**：返回合成的 `MessageMembers`，只填名称与大小，**不填字段偏移**。

关于 introspection 的安全性，已核对 `register_type_object` 的实现：它只读名称做类型名反查，
从不读 `offset_` 或数组访问器指针。因此合成结构不会崩。但也正因为反射信息不完整，
`ros2 topic echo` 无法工作（§5.2）——**这是同一个原因的两种后果**。

### 2.3 线格式

```
[0,2)    encapsulation kind    00 01 = CDR_LE / 00 00 = CDR_BE   ← Fast DDS 加的
[2,4)    encapsulation options
[4,8)    uint32 长度前缀（字节序随 encapsulation）                 ← 本方案 cdr_serialize 写的
[8,8+n)  protobuf 标准 wire format
[8+n,..) 4 字节对齐 padding
```

前 4 字节由 Fast DDS 添加，**不属于本方案**。实现 `_strip_length_prefix` 时曾漏掉这一层，
把 `00 01 00 00`（小端 256）当成长度，导致每帧误报「载荷过短」。已记录在 `echo.py` 注释中。

`cdr_serialize` / `cdr_deserialize` 只做「长度前缀 + 裸字节搬运」，不逐字段编解码。
因此 **protobuf 的线格式就是 DDS 载荷本身**，天然跨语言。

---

## 3. 需要替换什么

### 3.1 消息定义：7 个 `.msg` → 9 个 proto message

| 源 `.msg` | 字段数 | 对应 proto | 状态 |
|---|---|---|---|
| `ChassisLinkHealth.msg` | 14 | `chassis.proto` `ChassisLinkHealth` | 已实现 |
| `TrackingError.msg` | 11 | `chassis.proto` `TrackingError` | 已实现 |
| `DispatcherState.msg` | 19 | `dispatcher.proto` `DispatcherState` | 已实现 |
| `VehicleLinkState.msg` | 10 | `dispatcher.proto` `VehicleLinkState` | 已实现 |
| `SafetyState.msg` | 7 | `safety.proto` `SafetyState` | 已实现 |
| `TopologyState.msg` | 6 | `safety.proto` `TopologyState` | 已实现 |
| `MissionEvent.msg` | 7 | `safety.proto` `MissionEvent` | 已实现 |

另有 2 个共享结构：`common.proto` 的 `Time` 与 `Header`（替代 `std_msgs/Header`）。

**语义变更点（不是简单照搬）**：

- `ChassisLinkHealth.tier`：`.msg` 里是 `string`，proto 改为 `enum Tier`，
  另加 `tier_text` 保留无法映射的原始字符串。好处是拼写错误编译期就暴露。
- `ChassisLinkHealth.command_delivery_rate`：`.msg` 用 `-1` 表示「不适用」。
  proto3 无符号类型不能表达负数，故拆为 `command_delivery_rate_valid` + `command_delivery_rate`。
  **桥接代码负责翻译，下游消费者也必须改判断逻辑。**
- `SafetyState.severity`：同 `tier`，`enum` + `severity_text`。
- `Header` 是**重新定义**而非复用 `std_msgs/Header`。刻意为之：让 proto 类型不依赖任何
  ROS 消息包，从而在纯 protobuf 环境（如嵌入式端、工具链）中也能用。

### 3.2 C++ 源码：真实引用点

以下为 grep 实测结果（已排除 `install/`、`build/`）：

| 消息 | 引用处数 | 主要文件 |
|---|---|---|
| `MissionEvent` | 73 | `ros/dispatcher_node.*`、`core/workflow/*`（见下方警告） |
| `ChassisLinkHealth` | 34 | `ros/chassis_driver_node.*`、`link_health_publisher.cpp`、`safety_gate_node.*` |
| `SafetyState` | 31 | `ros/safety_gate_node.*`、`core/workflow/mission_safety_workflow.*` |
| `TrackingError` | 26 | `ros/path_tracker_node.*`、`navigate_sequence_bridge_node.*` |
| `DispatcherState` | 14 | `ros/dispatcher_node.*`、`tools/dispatcher_cli.cpp`、`rest_gateway.cpp` |
| `TopologyState` | 10 | `ros/dispatcher_node.*`、`dispatcher_visualizer_node.cpp`、`safety_gate_node.*` |
| `VehicleLinkState` | 2 | `ros/dispatcher_node.cpp` |

> **重要警告：`amr_dispatcher_core` 里也出现这些名字，但那是同名 struct，不是 ROS 消息。**
> 已核实：`amr_dispatcher_core/package.xml` **不依赖** `amr_dispatcher_interfaces`，
> 符合 ADR-001「核心算法与 ROS 完全解耦」。
>
> 因此迁移时**绝对不要动 `amr_dispatcher_core`**。真实要改的是
> `amr_dispatcher_ros`（节点层）与 `amr_dispatcher_tools`（CLI / REST）。
> 把「同名」误判为「同类型」是这次迁移最容易犯的错误，且编译期不一定报错
> ——两者字段恰好相似时会出现隐式转换成功但语义错误的静默 bug。

### 3.3 构建与依赖

| 项 | 变更 |
|---|---|
| `amr_dispatcher_ros/package.xml` | 加 `<depend>amr_dispatcher_proto</depend>` |
| `amr_dispatcher_ros/CMakeLists.txt` | `find_package(amr_dispatcher_proto REQUIRED)` |
| `amr_dispatcher_tools` | 同上（REST / CLI 输出需要 proto 类型） |
| 部署镜像 | 加 `libprotobuf-dev`（构建）与 `libprotobuf`（运行）+ `pybind11-dev` |

`amr_dispatcher_interfaces` **保留不删**。服务/动作仍完全依赖它，且过渡期要双发。

### 3.4 不改的东西（明确划界）

- `amr_dispatcher_core`：零改动（ADR-001）
- 24 个 `.srv`、2 个 `.action`：零改动（见 §5.1）
- 第三方消息（`std_msgs` / `nav_msgs` / `geometry_msgs` / `sensor_msgs` /
  `visualization_msgs` / `nav_msgs/OccupancyGrid`）：零改动。
  实测代码中 `std_msgs::msg::Bool` 用了 8 次订阅 + 4 次发布，
  这些标准化程度高、跨语言不是问题，不值得迁移。

---

## 4. 迁移策略：为什么不能「一次性切换」

### 4.1 DDS 类型名与兼容性

protobuf 类型的 DDS type name 由 `message_namespace_` + `message_name_` 决定
（在 `callbacks` 里设置），与 rosidl 生成的名字**不同**。
XTypes 类型匹配基于 type name + type hash，因此：

- 新旧两端**无法互通**：一个 protobuf 发布者和一个 rosidl 订阅者不会匹配
- 不存在「兼容解码」：类型不同就无法通信，与编码格式无关

### 4.2 双发过渡（推荐）

每个迁移的消息，过渡期内**同时发布两个话题**：

```
/chassis/link_health          ← 既有 rosidl 话题，保持不动
/chassis/link_health_proto    ← 新增 proto 话题
```

订阅方按自己的节奏迁移。全部迁完后删掉旧话题与旧 `.msg` 字段。

优点：任何时刻系统都是可回滚的；不影响现有调试流程与既有测试。
缺点：过渡期带宽翻倍（但 proto 话题本身更省，净增量有限）；发布端多一份代码。

### 4.3 全量切换（不推荐，但在小系统可行）

一次性改完所有生产者和消费者。省掉双份代码，但**没有并行验证期**，
一旦某个字段语义没对齐，只能整体回滚。本项目有 7 消息 + 24 srv + 2 action
+ 多个节点，风险偏高。

---

## 5. 边界与限制（不可回避）

### 5.1 服务与动作：明确不支持

服务（`.srv`）和动作（`.action`）的 typesupport 不是单个 handle，而是
request / response / event 的三元组结构。本方案的注入点只覆盖单个消息类型。

**结论：24 个服务、2 个动作永久保留 rosidl，不迁移。**
这部分占接口总数（33 个）的 79%。所以在可预见的未来，`amr_dispatcher_interfaces`
必须继续存在，本方案是「话题层可选替换」而非「整体替代 rosidl」。

### 5.2 工具生态：`ros2 topic echo` 不可用

根因链条（已逐层验证）：

```
echo.py
  → ros2topic.api.get_msg_class()
    → rosidl_runtime_py.utilities.get_message(type_name)
      → import_message_from_namespaced_type(...)      # 按名字 import Python 模块
        → is_message(interface) 判断 hasattr(interface, 'SLOT_TYPES')
```

protobuf 类型没有 rosidl 的 Python 生成包，`import` 必然失败。
这是**架构性**的：`ros2 topic echo` 依赖 Python 侧用同一份生成代码还原消息结构，
而 protobuf 走的是描述符池动态解析。写一个 echo 插件也绕不过去，
因为它不提供「换一个解析器」的钩子。

**已给出的解法：`ros2 amr echo`**

不试图修复 `ros2 topic echo`，而是新增一个 ROS CLI 命令组：

```bash
ros2 amr echo /chassis/link_health_proto amr_dispatcher.proto.ChassisLinkHealth --count 2
ros2 amr echo --list-types          # 列出全部可解析类型
```

实现要点：
1. 用 rclpy `raw=True` 订阅，直接拿 DDS 载荷 bytes，不构造 Python 消息对象
2. 从 `share/amr_dispatcher_proto/proto/descriptors.pb`（构建期生成）建描述符池，
   运行期**不需要 protoc**
3. `ParseFromString` 后用 protobuf 文本格式输出，字段名与 `.proto` 完全一致

其它工具的影响：

| 工具 | 状态 | 说明 |
|---|---|---|
| `ros2 topic echo` | ✗ | 架构性不可用（见上） |
| `ros2 topic hz` / `bw` | ✓ | 只看时间戳与字节数，不看内容 |
| `ros2 bag record` | △ | 能录，回放出来是裸 `uint8[]`，需插件才能还原 |
| RViz 可视化 | ✗ | 需要专门的 RViz 插件 |
| `rqt` 系列 | ✗ | 依赖 introspection 反射信息 |

### 5.3 反射信息不完整

合成的 introspection 只有名称、没有字段表。任何依赖反射的工具
（rqt、通用可视化、动态重组）都无法工作。这是 §2.2 权衡的直接后果：
为了让 rcl 不崩，必须提供 introspection；为了简单，返回的是最小实现。

若将来需要支持 rqt，可以生成**完整的** `MessageMembers`（字段名、类型、偏移都能从
`FileDescriptor` 推出）。工作量中等，但纯属可选增强。

### 5.4 其它已知限制

- **不支持嵌套超过 100 层的消息**：`max_serialized_size` 返回
  `ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE`，即无界类型。DDS 侧没有大小上限保护。
- **单帧上限 4 GiB**：长度前缀是 uint32。对遥测话题远远够用。
- **`max_serialized_size` 返回 0 且标记为 unbounded**：某些 DDS 实现可能对无界类型
  有额外限制（如 Fast DDS 的碎片化阈值）。当前 249 帧实测无问题，但大规模部署前
  建议做一次压力测试。
- **type hash 的稳定性**：当前 hash 只覆盖 `FileDescriptorProto`，不含 ROS 侧的类型名。
  若将来两个包定义了同名同结构的 proto，理论上 hash 会碰撞。建议后续把
  `message_namespace_` + `message_name_` 一并纳入 hash 输入。

---

## 6. 分阶段实施计划

### 阶段 0：骨架与验证 ✅ 已完成

- [x] `amr_dispatcher_proto` 包：proto 定义、typesupport 头、构建链路
- [x] `amr_dispatcher_proto_cli` 包：`ros2 amr echo`
- [x] 冒烟测试：C++ pub/sub 往返，字段逐项校验通过
- [x] pybind11 模块：把 typesupport handle 以 PyCapsule 暴露给 rclpy
- [x] 端到端：长驻发布者 + `ros2 amr echo`，解析出完整字段

验证记录：

```
$ ros2 amr echo /proto_smoke amr_dispatcher.proto.ChassisLinkHealth --count 2
--- [1] amr_dispatcher.proto.ChassisLinkHealth ---
header {
  stamp { sec: 123 nanosec: 456 }
  frame_id: "base_link"
}
backend_name: "serial"
tier: TIER_HEALTHY
frames_total: 1000000
frames_success: 999997
frames_crc_error: 3
is_healthy: true
```

### 阶段 1：垂直切片（1 个消息，单话题）

只迁 `ChassisLinkHealth`，双发 `/chassis/link_health` 与 `/chassis/link_health_proto`。

- [ ] `chassis_driver_node` / `link_health_publisher` 增加 proto 发布
- [ ] 保持既有 rosidl 发布不动
- [ ] 用 `ros2 amr echo` 与 `ros2 topic echo` 对照验证两者字段一致
- [ ] 跑现有测试套件，确认零回归

退出标准：proto 与 rosidl 两条链路数据逐字段一致；现有测试全绿。

### 阶段 2：核心遥测全量双发

`TrackingError`、`DispatcherState`、`SafetyState`、`TopologyState`、`MissionEvent`、`VehicleLinkState`。

- [ ] 逐个消息加 proto 发布
- [ ] 写一个「双发一致性检查器」：订阅两路，逐字段比对，差异即报错
- [ ] 该检查器接入 CI

退出标准：所有话题双发且一致性检查在 CI 中通过。

### 阶段 3：消费者迁移

- [ ] `amr_dispatcher_tools`（CLI / REST）改读 proto 话题
- [ ] 内部订阅方（`safety_gate_node`、`dispatcher_node` 等）迁移
- [ ] 每个消费者迁移后观察一个版本周期

### 阶段 4：清理

- [ ] 删除旧 rosidl 话题发布
- [ ] 删除 `amr_dispatcher_interfaces/msg/` 下已迁的 `.msg`
- [ ] 更新文档与运维手册

> **阶段 4 的前置条件**：确认没有任何外部系统（上位机、第三方集成）还在订阅旧话题。
> 这是整个计划里最容易被低估的风险——rosidl 话题名对第三方是公开契约。

---

## 7. 决策建议

本方案技术上可行且已验证，但**并不适合无脑推进**。建议按以下顺序决策：

**适合推进的信号：**
- 确实出现了跨语言消费需求（如 Rust 边缘节点、Go 的 Web 后端直接订阅）
- 遥测带宽成为瓶颈（先量化，别猜）
- 需要 proto 生态的工具链（schema 版本管理、代码生成）

**不适合推进的信号：**
- 团队重度依赖 rqt / RViz 排查问题——迁移后会失去这些工具
- 有第三方系统按当前话题名订阅——双发过渡期会拉长，且清理阶段有破坏风险
- 团队规模小、维护预算紧——多出的两个包、一套 CLI、一条绕开 ROS 工具链的调试路径，
  都是长期维护成本

**中肯结论：**
当前系统没有必须引入 protobuf 的硬性理由。这项工作的价值在于「打通了一条路」，
而不是「解决了当前痛点」。若决定推进，阶段 1 的双发切片风险最低，可以作为一个
低成本的可行性验证；若阶段 1 之后发现收益不明显，随时可以停在双发状态，
代价只是多维护一个 proto 包的发布逻辑。

---

## 8. 已交付的代码

### `src/amr_dispatcher_proto/`（C++ 包）

| 文件 | 作用 |
|---|---|
| `proto/*.proto` | 9 个消息定义（对应 7 个 `.msg` + 2 个共享结构） |
| `include/.../protobuf_typesupport.hpp` | 核心：typesupport 构造、四个 Jazzy 回调、CDR 编解码 |
| `include/.../register_protobuf_type.hpp` | `AMR_PROTO_REGISTER_TYPE(T)` 宏，一行完成注册 |
| `include/.../py_bindings.hpp` | `AMR_PROTO_BIND_TYPESUPPORT(...)` 宏 |
| `src/py_typesupport_module.cpp` | pybind11 模块，输出 PyCapsule |
| `scripts/gen_typesupport_bindings.py` | 从 `FileDescriptorSet` 生成绑定清单（正确处理嵌套消息） |
| `src/smoke_test.cpp` | 冒烟自检，支持 `--duration` 长驻模式 |

### `src/amr_dispatcher_proto_cli/`（Python 包）

| 文件 | 作用 |
|---|---|
| `amr_dispatcher_proto_cli/command.py` | `ros2 amr` 命令组 |
| `amr_dispatcher_proto_cli/echo_verb.py` | `echo` 子命令参数解析 |
| `amr_dispatcher_proto_cli/echo.py` | raw 订阅 + 布局解析 + protobuf 反序列化 |
| `amr_dispatcher_proto_cli/descriptor_pool.py` | 描述符池构建与缓存 |

未提交到 git（按约定，先看结构再决定）。

---

## 9. 复现步骤

```bash
# 构建
colcon build --packages-select amr_dispatcher_proto amr_dispatcher_proto_cli
source install/setup.bash

# 冒烟自检（2 秒后自退）
ros2 run amr_dispatcher_proto proto_smoke_test

# 长驻演示 + 解析
ros2 run amr_dispatcher_proto proto_smoke_test --duration 60 &
ros2 amr echo /proto_smoke amr_dispatcher.proto.ChassisLinkHealth --count 2
```

依赖：`libprotobuf-dev`、`protobuf-compiler`、`pybind11-dev`、`python3-protobuf`。
