# 硬件接入方案（Hardware Integration Plan）

> 状态：方案定稿，代码侧前置改造已完成并通过 971 项 core 测试（+ 114 tools / 27 ros）。
> 适用对象：Radxa Cubie A7A + ESP32 + JGB37-520 双轮差速台架。
> 本文只描述"怎么把真硬件接进现有软件"，不重复 README 的算法说明。

---

## 1. 结论先行

一句话：**不买整车，买 165 元的台架；不写新驱动体系，沿用现有 `ChassisBackend` 链路。**

三条边界要先说清楚，避免对成果有过高预期：

| 能力 | 台架能做到 | 台架做不到 |
| --- | --- | --- |
| 调度闭环（下单→出车→到站→接单） | ✅ 完整 | |
| 底盘 HAL / 协议 / CRC / 看门狗 | ✅ 完整，且是自己写的 | |
| 编码器里程计 + PID 闭环 | ✅ 完整 | |
| 物理定位（map→odom） | ⚠️ 静态偏移，需手动标定 | ❌ 无重定位、无 SLAM |
| 动态避障 | | ❌ 软件未接感知，也没有雷达 |
| 载重 / 爬坡 / 长时间无人值守 | | ❌ 免维护性未验证 |

主动说清这三行，比含糊地讲"我做了台 AGV"可信得多。

---

## 2. 硬件选型

### 2.1 已有

| 部件 | 规格 | 作用 |
| --- | --- | --- |
| Radxa Cubie A7A | ARM64，Debian | 车端 ROS 2 运行时（最贵的一块，省掉了） |
| 腾讯云 x86_64 | 已有容器 `amr_dispatcher` | 云端调度 / 算法 / REST（也可移到 A7A 本地） |

节点栈实测常驻内存合计 **57 MB**，`install/` 仅 17 MB，A7A 的算力严重过剩——瓶颈不在板子。

### 2.2 待购（双轮差速，约 163 元）

| 物料 | 搜索词 | 数量 | 单价 | 小计 |
| --- | --- | --- | --- | --- |
| MCU | `ESP32开发板 Type-C 38P` | 1 | 18 | 18 |
| 编码器电机 | `JGB37-520 编码器电机 12V` 1:30，**必须 6 线带 AB 相** | 2 | 32 | 64 |
| 电机支架 | `37电机支架 L型` | 2 | 4 | 8 |
| 驱动 | `TB6612FNG 电机驱动模块`（排针已焊） | 1 | 8 | 8 |
| 电源 | `12V 5A 电源适配器 DC5.5` | 1 | 25 | 25 |
| 电源转接 | `DC5.5 母头转接线柱` | 1 | 3 | 3 |
| 杜邦线 | `杜邦线 20cm` 公母各一排 | 1 | 5 | 5 |
| 急停 | `22mm 急停开关 常闭 蘑菇头` | 1 | 10 | 10 |
| 底板 | 亚克力/洞洞板 约 10×15cm | 1 | 12 | 12 |
| 万向轮 | `牛眼轮` 2 只 | 1 | 8 | 8 |
| 电阻 | 10k + 20k 各两只（备用分压） | 1 | 1 | 1 |

**预算紧张就买 96 元的单轮版**（去掉一只电机、一个支架、底板、万向轮），先验证 MCU + 协议 + 编码器这一层。单轮跑不了差速 tracker，但能跑通全部通信与安全链路，不是白买。

### 2.3 明确不买

- **ST-Link**：ESP32 自带 USB 下载。
- **74HC4050 / TXS0108E**：编码器 VCC 直接接 3.3V，输出即 3.3V，可直连。真遇到波形不稳再用两只 10k+20k 分压（成本 0.2 元）。
- **OLED / 逻辑分析仪 / 第二张 SD 卡**：演示价值低，先不花。
- **阿克曼底盘**：代码里 `NormalizeKinematicsModel` 把 `ackermann` 直接映射成 `diff_drive`，等于不支持。
- **CAN 总线底盘**：`ChassisBackend` 只有 serial / udp 两个实现，接 CAN 要重写。
- **激光雷达 / RTK / UWB**：站点式调度用不上，且软件没有感知消费方。

**选型硬约束**（来自代码，不是拍脑袋）：

- 电压：`kMinimumBatteryVoltage` 与 `fleet.yaml: 24.5` → 24V 系统。**台架用 12V 时必须把 `min_battery_voltage` 改成 10.8**，否则任务永远判电量不足。
- 轮径/轮距：`wheel_radius 0.08`、`wheel_separation 0.45` → 160mm 轮、0.45m 轮距，JGB37-520 正好。
- 速度：`max_linear_x_mps 1.5`，tracker 默认 `target_speed 0.5`。
- 场地：demo map 13×7m、通道宽 1.2m → 车宽建议 ≤ 0.7m。

---

## 3. 拓扑与供电

```text
        [云端 / PC]  REST + DDS
              │ Wi-Fi
              ▼
   ┌──────────────────────────────┐
   │ Radxa Cubie A7A (ARM64)      │  5V/3A USB-C 独立供电
   │ /dev/ttyAS4  ← UART4 overlay │
   └───────────┬──────────────────┘
               │ TTL 3.3V  460800 8N1  TX/RX 交叉 + 单点共地
   ┌───────────▼──────────────────┐
   │ ESP32  1kHz 控制环            │  ← 编码器捕获 / 增量 PID / 看门狗
   └────┬──────────────────┬──────┘
        │ PWM + DIR        │ A/B 相
   ┌────▼──────┐    ┌──────▼──────┐
   │ TB6612FNG │    │ 霍尔编码器×2 │
   └────┬──────┘    └─────────────┘
        │ VM 12V
   ┌────▼──────────────────────────┐
   │ 12V/5A ── [急停常闭] ── 电机   │  急停物理切断动力，不依赖软件
   └───────────────────────────────┘
```

要点：

- **双电源、单点共地**。12V 给电机，5V/3A 给 A7A，两路 GND 必须相连，否则 UART 没有电平参考。
- 12V 入口并 470µF + 100nF，吸收电机启动浪涌。
- **不要用 12V/2A**：两只 JGB37-520 堵转合计 3–5A，2A 会掉压重启。
- **急停串在 12V 的 VM 回路上**——按下就断动力，与软件完全无关。

### 3.1 A7A 引脚

| A7A 40-pin | 信号 | 接到 ESP32 | 备注 |
| --- | --- | --- | --- |
| 16 | PJ24 / UART4_TX | RX (GPIO16) | 交叉 |
| 18 | PJ25 / UART4_RX | TX (GPIO17) | 交叉 |
| 14 或 20 | GND | GND | 只接一根 |
| 17 | +3.3V | — | **不要**给电机供电 |

UART4 默认关闭：`rsetup` → Overlay → Manage overlays → 勾选 **Enable UART4** → 重启后出现 `/dev/ttyAS4`。

> **关键坑**：`entrypoint.server.sh` 默认 `AMR_SERIAL_DEVICE=/dev/ttyUSB0`。A7A 板载串口叫 `/dev/ttyAS4`，不指定就会去起 PTY 模拟器，然后"系统看着正常、车一动不动"。

---

## 4. 通信协议：沿用 Mick 二进制帧

**不要新造协议**。仓库已有 `EncodeMickBinaryFrame` / `DecodeMickBinaryPacket` 及其单测（`test_chassis_packet.cpp`）。MCU 直接实现这套帧，编解码、测试、粘包处理全部现成。

### 4.1 帧格式

```text
0xAE 0xEA | len | type | payload... | sum8 | 0xEF 0xFE
```

`sum8` = 从 `len` 到 payload 末字节的**累加和（uint8 溢出）**。

### 4.2 下行（A7A → ESP32）

`type = 0xF3`，payload 6 字节，三个 int16（大端，单位 mm/s、mrad/s）：

| 偏移 | 字段 | 说明 |
| --- | --- | --- |
| 0–1 | linear_x | ×1000，mm/s |
| 2–3 | linear_y | ×1000，差速底盘恒 0 |
| 4–5 | angular_z | ×1000，mrad/s |

清除里程计：`type = 0xE1`，payload `01 00 00 00`。

### 4.3 上行（ESP32 → A7A）

**里程计 `type = 0xA1`，payload ≥ 12 字节**：

| 偏移 | 字段 | 单位/含义 |
| --- | --- | --- |
| 0–1 | x | mm（int16） |
| 2–3 | y | mm |
| 4–5 | yaw | mrad |
| 6–7 | vx | mm/s |
| 8–9 | wz | mrad/s |
| 10–11 | battery | ×100，即 12.34V → 1234 |

**状态 `type = 0xA2`**：`payload[0]` 0=正常/非0=fault；`payload[1..2]` 电压 ×100。

> int16 表示 x 的极限是 ±32.767 m。台架够用；大场地需要扩成 int32，届时同步改 `chassis_packet.cpp`。

### 4.4 时间尺度

| 层 | 周期 | 职责 |
| --- | --- | --- |
| MCU 控制环 | 1 ms（1 kHz） | 读编码器 → 算实际速度 → 增量 PID → 更新 PWM |
| MCU 上报 | 20 ms（50 Hz） | 发 0xA1 |
| A7A 轮询 | 20 ms（50 Hz，`poll_rate_hz`） | 解析帧、发 odom / TF |
| A7A 看门狗 | 300 ms（`odom_timeout_ms`） | 无有效帧 → 心跳置 false |
| 安全门看门狗 | 500 ms（`heartbeat_timeout_ms`） | 心跳丢失 → 零速 |

带宽：50 Hz × 20 字节 ≈ 1 KB/s，115200 都够；用 460800 可以把单帧传输降到 0.23 ms。

### 4.5 双向看门狗

- **MCU 侧**：连续 200 ms 没收到合法下行帧（或 sum8 错）→ PWM 归零 + 拉高制动，状态位 `uart_timeout`。
- **A7A 侧**：300 ms 没有有效上行帧 → `link_alive_ = false` → `ChassisLinkHealth.is_healthy = false`、心跳发 `false` → `safety_gate` 的 500 ms 看门狗接管零速。

这两级是"拔线演示"能成立的基础，缺一不可。

---

## 5. 软件接入路径：走 `ChassisBackend`，不走 ros2_control

**选择理由**（这是本方案最重要的工程判断）：

| | `chassis_driver_node` + `ChassisBackend` | ros2_control `SystemInterface` |
| --- | --- | --- |
| 现状 | 主链路，`safety_gate`/`path_tracker` 都在用 | 孤岛：无 URDF、无控制器 yaml、launch 里 `parameters=[]` |
| 改造成本 | 已改完（`protocol` 参数 + 二进制解析） | 需补 URDF、控制器配置、位置来源改造 |
| 位置精度 | 直接用底盘上报的 x/y/yaw | 现在是轮速积分累加，会漂 |
| 结论 | ✅ 采用 | ❌ 暂不引入 |

`ros2_control` 的 `chassis_hardware_interface.cpp` 保留不动，作为未来接 nav2 生态的入口；当前不启用。

---

## 6. 已完成的代码改造

以下改动均已落地，容器内编译通过，`ctest` **971/971 通过**（新增 6 项 `BatteryVoltageTest`）。

> 第 8-11 条不是设计时想到的，是 PTY 端到端联调时暴露的：只跑单元测试会全部通过，因为单测直接喂完整帧，绕过了真实串口的半帧、空读与时序。

| # | 问题 | 改动 |
| --- | --- | --- |
| 1 | 下行按协议发、上行永远按文本切，接二进制底盘后 odom 全丢 | `chassis_driver_node` 新增 `protocol` 参数（`text` / `mick_binary`），`IngestBytes()` 按协议分派到 `MickBinaryPacketStream` 或 `ChassisPacketStream`，并把 `protocol` 传给 `ChassisBackendConfig` |
| 2 | 底盘电压解析出来就被丢弃 | 新增 `/battery_state` 发布（`sensor_msgs/BatteryState`），含电压与百分比，按 0.05V 变化节流 |
| 3 | 百分比端点写死，12V/24V 无法共存 | 新增 `battery_full_voltage` / `battery_empty_voltage` 参数；换算抽成 `battery_voltage.hpp` 纯函数 + 6 项单测 |
| 4 | `kMinimumBatteryVoltage = 21.0` 写死，12V 台架永远判电量不足 | 改为 `min_battery_voltage` / `battery_drop_per_meter` 参数，默认值保持 21.0（不改既有 24V 行为） |
| 5 | `PublishHealth` 无条件发 `heartbeat=true`，拔线降级到 mock 后永远不刹车 | 心跳与 `is_healthy` 改为依赖 `link_alive_`（有有效上行帧），并新增 `odom_timeout_ms` |
| 6 | `entrypoint.server.sh` 设备不存在时静默起 PTY 模拟器 | `AMR_USE_MOCK_CHASSIS=0` 且设备不存在 → 打印原因并 `exit 1` |
| 7 | `/plan` 是 map 帧，`/odom` 原点不一定是 map 原点，真机会朝反方向开 | `path_tracker_node` 新增 `odom_origin_x/y/yaw` 静态偏移（默认 0，仿真行为逐位不变），并对非 map 帧的路径告警 |
| 8 | 二进制模式下空读被计为丢包，启动后 8 个样本就把 `loss_rate` 顶到 1.0、后端降级到 mock | 空读仅在文本协议下计损；链路死亡改由 `odom_timeout_ms` 判定 |
| 9 | 坏帧用 `RecordLoss` 计，解码噪声会误判成链路中断 | 改用 `RecordCrcError()`（只污染滑窗、同时刷新 `last_seen`） |
| 10 | 超时检测写在 `Read()` 成功路径之后，空读提前 `return` 导致假活永不触发（拔线不报警） | 抽出 `EvaluateLinkLiveness()`，两条退出路径都调用 |
| 11 | 电压恒定时节流把 `battery_state` 彻底静默，下游拿不到首帧 | 变化 >=0.05V 立即发，否则每 2s 保底发一次 |

对应参数（`config/chassis.yaml` 已写入；仓库里的默认值是 **text + 12V 台架端点**，
接真 MCU 时才改 `protocol`）：

```yaml
protocol: "text"                 # 接 MCU 时改 mick_binary（两侧必须一致）
battery_full_voltage: 12.6       # 12V 台架；24V 整车改 25.2
battery_empty_voltage: 11.0      # 24V 整车改 21.0
odom_timeout_ms: 300             # 必须小于 safety_gate 的 500ms 心跳超时
```

注意 `protocol: "text"` 是**默认值而不是待办**：仿真链路（PTY 模拟底盘）只有文本协议，
新装机的镜像必须能开箱即用。

`config/dispatcher.yaml`：

```yaml
min_battery_voltage: 10.8        # 12V 台架；24V 整车保持 21.0
```

---

## 7. 落地路线

| 阶段 | 目标 | 验收标准 | 预估 |
| --- | --- | --- | --- |
| P0 | A7A 开 UART4 overlay | 短接 PJ24/PJ25 能回环收发 | 2h |
| P1 | ESP32 单机闭环 | 串口给目标转速，电机稳转，手捏阻力能补 PWM | 4h |
| P2 | 双向帧协议 | 拔线 200ms 内零速；sum8 错帧被丢弃并计数 | 4h |
| P3 | 接入 ROS | `ros2 topic echo /odom` 有真实位姿；`/battery_state` 有电压 | 3h |
| P4 | 调度闭环 | Web 下单 → 车真的开到站点 → `arrived` → 接下一单 | 4h |
| P5 | 故障矩阵 | 拔线 / `kill -9` / 错帧 / 按急停，四种故障行为符合预期 | 4h |

**P5 是重点产出**：一张"故障注入 × 预期行为 × 实测响应时间 × 恢复方式"的表，比任何形容词都硬。建议直接放进 README。

---

## 8. 故障矩阵（待实测填写）

| 注入手段 | 预期行为 | 实测响应 | 恢复方式 |
| --- | --- | --- | --- |
| 拔 UART 线 | MCU 200ms 内零速；A7A 300ms 判假活；安全门 500ms 零速 | | 插回自动重连 |
| `kill -9` chassis_driver | 心跳停 → 安全门零速 | | 重启节点 |
| 注入 sum8 错帧 | 帧被丢弃，`frames_lost` 递增，不影响后续帧 | | 自动 |
| 按下急停 | 物理断 12V，电机立刻失力 | | 旋出急停 |
| 暂停上游 cmd_vel | 500ms 后驱动主动写零帧兜底 | | 恢复指令即解除 |
| 底盘掉电重启 | odom 归零 → 需重新标定 `odom_origin_*` | | 重新标定 |

---

## 9. Debian 上部署 ROS 2

A7A 官方只有 Debian / Android，没有 Ubuntu；`ros-jazzy-*` 只针对 noble 构建，直接 `apt install` 会失败。

**采用 Docker**（与仓库现有容器化一致）：

```bash
# PC 上交叉出 arm64 镜像（不要在板子上编译）
docker buildx build --platform linux/arm64 \
  -f docker/Dockerfile.jazzy -t amr-dispatcher:jazzy-arm64 --load .

# A7A 上运行：透传串口 + host 网络让 DDS 直连
docker run -d --name amr_dispatcher \
  --network host \
  --device /dev/ttyAS4:/dev/ttyAS4 \
  -e AMR_USE_MOCK_CHASSIS=0 \
  -e AMR_SERIAL_DEVICE=/dev/ttyAS4 \
  -v /home/radxa/amr_dispatcher:/workspace \
  amr-dispatcher:jazzy-arm64
```

- 板子 **不要跑 RViz**，headless 即可；PC 上用 RViz2 或 Foxglove 看。
- 小内存机器编译有 OOM 风险，镜像在 PC 上构建好再传。
- 若 A7A 上直接跑调度（单机闭环，不跨公网 DDS），把 `dispatcher_node` 也放进同一容器最简单。

---

## 10. 常见坑

1. **共地只接一根**：多点接地会形成地环路，编码器噪声窜进 UART。
2. **编码器线不屏蔽**：PWM 是强干扰源，A/B 用双绞线并远离动力线。
3. **16 位计数器回绕**：定时器编码器模式读累计值，1kHz 取增量时必须处理回绕，否则速度突然跳成负值。
4. **SD 卡写坏**：ROS 日志写到 `tmpfs`，挂载加 `noatime`。
5. **忘记改 `min_battery_voltage`**：12V 台架上所有任务会被判"电量不足"。
6. **忘记指定 `AMR_SERIAL_DEVICE`**：会静默走 PTY 模拟器（已改为显式报错）。
7. **`odom_origin_*` 不标定**：车会朝坐标原点方向跑偏，表现为"往反方向开"。