# 台架硬件接入实操手册（Bench Hardware Bring-Up Runbook）

> 配套文档：`HARDWARE_INTEGRATION.md`（选型、决策依据、待办清单）。
> 本文是**照着敲就能跑通**的施工顺序，所有命令、引脚、帧偏移都对齐仓库现有代码。
> 台架：Radxa Cubie A7A + ESP32 + 2× JGB37-520 + TB6612FNG。
>
> **v2 修订（2026-10-30）**：修正分压电阻、减速比、底板尺寸、测速窗口、yaw 归一化等
> 会导致烧件或白跑的问题。变更清单见文末 §10。
>
> **v2.1 修订（2026-11-04）**：T4–T7/T9 落地后，台架不再需要手改任何 yaml ——
> 协议/波特率/原点/容差全部走环境变量（见 §0 表格与 §5.1）。
>
> **v3 修订（2026-11-07）**：新增「路线 B：成品底盘」。机械省下的时间必须换成
> **§3.4 的四项参数实测**，否则轮径填错会让车冲过站点接近一倍。变更清单见文末 §10。

---

## 0. 术语与事实基线

| 项 | 值 | 来源 |
| --- | --- | --- |
| 板载串口 | `/dev/ttyAS4`（PJ24/PJ25，需开 UART4 overlay） | Radxa 官方文档 |
| 帧头/帧尾 | `0xAE 0xEA` / `0xEF 0xFE` | `chassis_packet.cpp:EncodeMickBinaryFrame` |
| 下行速度帧 | `type=0xF3`，payload **7B**：linear_x/linear_y/angular_z 各 int16 BE ×1000 + **seq** uint8。**固件必须同时接受 6B（旧）和 7B（新）两种** | `EncodeMickSpeedCommand` |
| 清里程计帧 | `type=0xE1`，payload `01 00 00 00` | `EncodeMickClearOdometryCommand` |
| 上行里程计 | `type=0xA1`，payload **≥12B**：x,y,yaw,vx,wz(mm/mrad) + battery(×100)；**可选扩展**：第 12 字节 flags（bit0 fault/bit1 uart_timeout/bit2 estop/bit3 overcurrent）、第 13 字节 last_cmd_seq | `DecodeMickBinaryPacket` |
| 上行状态 | `type=0xA2`，`payload[0]` 非0=fault（**会触发停车**），`payload[1..2]` 电压 ×100 | 同上 |
| 校验 | `sum8`：从 `len` 字节累加到 payload 末字节，uint8 溢出 | `EncodeMickBinaryFrame` |
| 链路参数 | `serial_baud=115200`、`protocol="text"` 为仓库默认；**可从 launch/环境变量覆盖**（T5） | `config/chassis.yaml` / `full_system.launch.py` |
| 接入后端 | `ChassisBackend`（serial/udp），**不是** `ros2_control` | `chassis_driver_node.cpp` |
| 看门狗 | `odom_timeout_ms=300` < `heartbeat_timeout_ms=500` | `chassis.yaml` / `safety.yaml` |
| 到达容差 | 默认 **0.10 m**，**可配置**：`goal_tolerance:=0.3` 或 `AMR_GOAL_TOLERANCE=0.3`（T4） | `path_tracker_node` 参数 |

两个默认值必须先处理，否则台架一上电就"行为正常但车不动"：

1. `protocol`：默认 `text` → 必须是 `mick_binary`。
2. `serial_baud`：默认 `115200` → 与固件一致（建议 `460800`，见 P2）。

**推荐用环境变量，不要改 yaml**（T5 已落地）：

```bash
-e AMR_CHASSIS_PROTOCOL=mick_binary -e AMR_CHASSIS_BAUD=460800
```

这样仓库默认值保持 `text`/`115200`（PTY 仿真链路靠它开箱即用），演示完也不必记得改回来。
**固件侧波特率必须同步，两侧不一致的表现是"一帧都收不到"，不是"收到乱码"。**

---

## 1. 采购与到货清点

清单见 `HARDWARE_INTEGRATION.md` 第 3.2 节（路线 A：散件组装，约 164 元）与
第 3.2b 节（路线 B：成品底盘，约 180 元）。**先选定路线，再按对应清单清点。**

两条路线的差别只有机械部分——「编码器电机 ×2 + L 型支架 ×2 + 底板 + 牛眼轮」（约 92 元）
换成一套成品底盘，**其余物料两条路线完全相同**。

### 1.1 共同项（两条路线都要）

- [ ] ESP32 开发板 ×1（Type-C，38P，**确认是 WROOM 不是 WROVER**——WROVER 的 GPIO16/17 被 PSRAM 占用）
- [ ] TB6612FNG 模块 ×1（排针已焊）
- [ ] 12V/5A 适配器 ×1 + DC5.5 母头转接线柱 ×1
- [ ] 急停开关 ×1（常闭）
- [ ] 电阻 **100k ×1 + 10k ×5**（1 只分压高端 + 1 只分压低端 + **4 只编码器上拉**；**不是 10k/20k**，见 §2.4）
- [ ] 有源蜂鸣器 3.3V ×1（看门狗触发时响，视频里可听可见）
- [ ] 杜邦线（公母各一排）——**仅信号线用，动力线另买**
- [ ] A7A 的 5V/3A USB-C 电源（**不要用 12V 适配器给板子供电**）
- [ ] A7A↔ESP32 杜邦线三根（TX/RX/GND）
- [ ] **18AWG 硅胶线 1 m（红黑各一）**：`12V+ → 急停 → TB6612 VM` 走 3–5A，杜邦线约 1A 额定会发热
- [ ] **5A 自恢复保险丝 ×1**：串在 12V 正极
- [ ] 5V/1A USB 充电头（ESP32 独立供电）

### 1.2 路线 A 专属（散件组装）

- [ ] JGB37-520 编码器电机 ×2（**1:60 减速比**，**6 线：2 动力 + 2 编码器电源 + 2 AB 相**）
- [ ] 电机支架（L 型）×2
- [ ] 底板 **50×20cm**（**别按 v1 的 10×15cm 买**：轮距 0.45 m 的车放不进去）
- [ ] 牛眼轮 ×2

### 1.3 路线 B 专属（成品底盘）

- [ ] JGB37-520 双轮差速成品底盘 ×1 套（自带底板 + 万向轮 + 电机）

**下单前必须问客服四句**（详见 `HARDWARE_INTEGRATION.md` §3.2b）：

1. **轮子直径多少？** → 决定减速比与 §3.4 标定。Ø65 配 1:60 时 0.5 m/s 要 86% 占空比，跑不到。
2. **编码器 VCC 支持 3.3V 吗？** → 只能 5V 时 AB 输出 5V，**直连 GPIO34/35 会打坏 ESP32**，必须加 74HC4050。
3. **编码器是 AB 相还是单相？** → 单相无法做里程计。
4. **电机线 6 线还是 5 线？** → 接法完全不同。

- [ ] 74HC4050 电平转换模块 ×1（**若客服说不清编码器供电，直接买，几块钱买保险**）

> **路线 B 到货后先验线序再接线**：不同厂家颜色定义不一致。万用表电阻档量，
> 两根几欧（2–10 Ω）的是电机动力线，剩下四根是编码器。详见 `HARDWARE_INTEGRATION.md` §3.2b。

---

## 2. 阶段 P0：A7A 自身先跑通（不接任何电机）

目标：证明板子、系统、串口、ROS 2 运行时都是好的。**这一步不碰 12V，不会有任何损坏风险。**

### 2.1 开 UART4

```bash
# A7A 上
sudo rsetup
# 依次进入：Overlay -> Manage overlays -> 勾选 Enable UART4 -> 确认 -> 重启
sudo reboot
```

重启后确认设备节点存在：

```bash
ls -l /dev/ttyAS4
# 期望：crw-rw---- 1 root dialout ...
sudo usermod -aG dialout $USER   # 当前用户加入 dialout，重新登录生效
```

### 2.2 硬件回环自检（关键一步）

用一根杜邦线**短接 PJ24（pin 16）和 PJ25（pin 18）**，然后：

```bash
python3 - <<'PY'
import serial, time
s = serial.Serial('/dev/ttyAS4', 115200, timeout=0.5)
s.write(b'loopback-test\n')
time.sleep(0.1)
print('RX:', s.read(64))
s.close()
PY
```

期望打印 `RX: b'loopback-test\n'`。**如果这里失败，先别往下走**——大概率是 overlay 没生效，或引脚搞错（对照 `HARDWARE_INTEGRATION.md` 4.1 的引脚表）。

测完把短接线拆掉。

### 2.3 确认 ROS 2 运行时

A7A 是 Debian，不能直接 `apt install ros-jazzy-*`。检查是否已有可用容器：

```bash
docker images | grep -i amr
docker ps -a | grep -i amr
```

没有镜像就先在 PC 上构建并传输（详见 `HARDWARE_INTEGRATION.md` §13）。**不要试图在 1.4GiB 内存的云服务器上编译。**

### 2.4 先做分压，再上电（顺序很重要）

**分压电阻算错会直接烧 ESP32 的 ADC**，所以这一步放在接电机之前、单独验证。

按 **100k / 10k** 接（分压比 1/11）：

| 输入电压 | 分压输出 | 是否安全 |
| --- | --- | --- |
| 12.6V（12V 台架满电） | 1.15V | ✅ |
| 24V（整车） | 2.18V | ✅ |
| 30V（极限） | 2.73V | ✅ |
| ~~10k/20k 分压 @12.6V~~ | ~~8.40V~~ | ❌ **烧 ADC** |

```text
12V+ ──[100k]──┬──[10k]── GND
               │
               └── 100nF ── GND
               │
               └── GPIO32 (ADC1_CH4)
```

要点：

- **必须走 ADC1（GPIO32–39）**，ADC2 与 WiFi 冲突；
- 100nF 对地不能省（高阻源 + 电机噪声会让读数乱跳）；
- ESP32 的 ADC 非线性，**必须两点标定**：万用表读 12.0V 和 11.0V，把两点的 raw 值写进固件的线性换算；
- 用可调电源从 10V 扫到 13V，确认读数线性、不饱和。

> **电阻数量（双电机共 5 只 10k）**：分压用掉 100k ×1 + 10k ×1，剩下 **10k ×4** 全部给
> 编码器 A/B 上拉——双电机有 4 路信号，**每路都要独立上拉到 3.3V**。只买 2 只的话，
> 双电机阶段会有一半信号没上拉，表现为「一个轮子转、另一个的 odom 不动」或直行时 odom 往一侧偏。
> 详见 `HARDWARE_INTEGRATION.md` §3.4 的数量表。

### 2.5 P0 验收

- [ ] `/dev/ttyAS4` 存在且当前用户在 `dialout` 组
- [ ] 短接回环能收到自己发的字节
- [ ] 分压在 12.6V 时输出 **1.15V ± 0.1V**（万用表实测）
- [ ] `docker run --rm amr-dispatcher:jazzy-arm64 ros2 --version` 正常输出版本
- [ ] A7A 上 `cat /proc/cpuinfo | head -2` 显示 aarch64

---

## 3. 阶段 P1：ESP32 单机闭环（不接 A7A）

目标：电机能按目标转速稳定转，轻阻能自动补 PWM。这一层把**硬件本体**调通，后面才有资格谈协议。

### 3.1 接线（先只接一个电机）

| 连接 | 说明 |
| --- | --- |
| 12V+ → 急停 → TB6612 `VM` | 急停必须串在动力回路里 |
| 12V− → TB6612 `GND` | **与 ESP32 GND 共地** |
| TB6612 `VCC` → ESP32 3.3V | 逻辑供电 |
| TB6612 `A01/A02` → 电机 M+ / M− | 电机动力 |
| TB6612 `PWMA` → GPIO25 | PWM |
| TB6612 `AIN1/AIN2` → GPIO26/27 | 方向 |
| TB6612 `STBY` → 3.3V | 否则不出力 |
| 编码器 `VCC` → ESP32 **3.3V** | **仅当编码器确认支持 3.3V 供电**（§1.3 第 2 问）。若标称 5V：VCC 接 5V，A/B 经 74HC4050 降到 3.3V |
| 编码器 `GND` → ESP32 GND | |
| 编码器 `A/B` → GPIO34/35（或经 4050） | 输入专用脚，内部无上拉，**需外接 10k 上拉到 3.3V**。**上拉必须拉 3.3V，不能拉 5V** |
| 分压中点 → GPIO32 | 见 §2.4 |

> **ESP32 的供电**：用 USB 线接 PC 或 5V/1A 充电头。**不要接 A7A 的 40-pin 3.3V**——
> ESP32 开 WiFi 峰值 200mA 以上，那一路不是给这个用的。接 PC 还有个额外好处：
> 串口监视器能打出**微秒级时间戳**，这是 P5 测停机延迟最可靠的手段。
>
> **共地要连三处**：12V 电源 GND、ESP32 GND、A7A GND。ESP32 走 PC 的 USB 供电时，
> **PC 的 GND 也要并进来**。漏了会表现为「收到乱码但解不出帧」，很容易误判成波特率不对。
>
> **动力线不要用杜邦线**：`12V+ → 急停 → TB6612 VM` 走 3–5A，杜邦线约 1A 额定会发热。
> 用 18AWG 硅胶线；动力线也不上面包板。
>
> **TB6612 的余量**：每通道连续 1.2A、峰值 3.2A，而单只电机堵转 2–3A。
> 轻载巡航（0.2–0.5 m/s）约 0.3–0.6A/只，没问题；但**验收时不要长时间捏死输出轴**，
> 轻阻 1–2 秒即可，否则模块会热保护。**成品底盘整机更重，余量更吃紧。**

### 3.2 固件骨架（Arduino / ESP-IDF 皆可）

顺序照抄即可：

1. GPIO25 配 LEDC PWM（20kHz，8bit）。
2. GPIO34/35 配输入 + CHANGE 中断，ISR 里只做 `encoder_count++/--`，**不要在里面算浮点**。
3. 1kHz 定时器：取 `count` 副本并清零，**累加到 20ms 窗口**。
4. **20ms 窗口到期才更新一次速度**：
   ```
   rpm = 窗口计数 / (PPR * 4 * gear) * 60 / 0.02
   ```
   `PPR` 按实物铭牌（常见 11 线），`gear` 按减速比（1:60 → 60）。
5. 同一个 20ms 节拍上跑增量式 PID：`pwm += Kp*(e-e1) + Ki*e + Kd*(e-2e1+e2)`，限幅 `[0,255]`。
6. 串口 100Hz 打印 `rpm` 便于观察（这阶段用文本，方便调试）。

> **为什么不是 1kHz 测速**：0.5 m/s、Ø160 轮约 59.7 rpm，1:60 编码器 2640 counts/圈。
> 1ms 窗口只有 2.6 个计数（误差 ~38%），20ms 窗口有 52.5 个计数（误差 ~1.9%）。
> 1kHz 保留给 PWM 更新，测速窗口放宽到 20ms。**这是 v1 最容易实现错的一条。**
>
> **成品底盘（Ø65 轮）的对应值更好**：0.5 m/s 时输出轴约 146.8 rpm，
> 20ms 窗口有 129 个计数（误差 **0.77%**）。所以 20ms 窗口在两条路线上都成立——
> 但注意这解决的是**测速精度**，不解决 §3.4 的**占空比余量**（Ø65 配 1:60 跑 0.5 m/s 要 86% 占空比）。

### 3.3 P1 验收

- [ ] 给定 `target_rpm=60`，实测稳定在 **±5%** 以内
- [ ] 轻阻 1–2 秒，PWM 明显上升，松手后回到目标值且不过冲震荡
- [ ] 按急停，电机立刻失力（不依赖任何软件）
- [ ] 换第二个电机重复上述步骤
- [ ] 串口打印的 rpm 与手持转速计（或用手机慢动作数圈）误差 <10%

**常见失败**：方向反了（交换 M+/M−）、编码器不计数（上拉电阻没接 / 接错脚）、一给 PWM 就重启（12V 没共地或电源功率不够）。

### 3.4 机械参数实测（成品底盘必做，散件组装同样要做）

**这一步不要跳。** 成品底盘省掉的是打孔和调平，**不是标定**。下面四个数填错不会报错，
只会让车到不了站——而症状很像 `goal_tolerance` 或调度问题，排查会绕很久。

| # | 参数 | 怎么测 | 填错的表现 |
| --- | --- | --- | --- |
| 1 | **轮径** | 卡尺量直径；或地上滚 10 圈量总距离 ÷ 10 ÷ π（含轮胎压缩，更准） | 距离成比例偏差。填大 → 车实际比报告慢 → **冲过站点** |
| 2 | **轮距** | 两轮接地中心距（卡尺） | 角速度成比例偏差 → 转角后航向累积误差 |
| 3 | **PPR（线数）** | 手转输出轴 10 圈，读计数 ÷ 10 ÷ 4（四倍频） | 与 #1 叠加的比例错误，两个都错时极难定位 |
| 4 | **A/B 相序 + 电机极性** | 手向前推车，看计数增减；正方向推应为增 | 「下发前进、车后退」。**必须在 P1 修掉**，到 P3 才发现会被误当成 `odom_origin` 没标定 |

**顺序（不要跳）**：

```text
① 静态量轮径 / 轮距（卡尺，不上电）
② 手转轮子定 PPR 与相序（不上电，最安全）
③ 本阶段：target_rpm=60 实测 ±5%（此时还不需要轮径）
④ P3 接 ROS 后：下发 0.3 m/s 走 5 秒，实测位移 / 5 应 ≈ 0.3，偏差 >5% 回头修 #1
⑤ P3 接 ROS 后：原地转 90°，看 odom 的 yaw 偏差 → 修 #2
```

> **为什么强调这一节**：`HARDWARE_INTEGRATION.md` §4.1 里的 `wheel_radius = 0.08 m`、
> `wheel_separation = 0.45 m` 是**散件组装路线的示例值**，不是默认值，更不是可直接烧进固件的常量。
>
> 成品底盘常见 **Ø65 轮 + 轮距 0.10–0.20 m**。若固件照抄 0.08 m，`odom` 报告的距离
> 只有实际的 **40%**，车会跑到站点接近两倍远才停。**文档里的任何示例值都要用自己的实测替换。**
>
> 顺带一句：成品底盘轮距窄，原地转向时轮胎横向刮蹭大，`odom` 的 yaw 漂移会比宽轮距方案明显。
> 这直接抬高了对 **F6（掉电重启）** 的依赖——`HARDWARE_INTEGRATION.md` §12 的
> "摆回标记点 + 重启节点"流程在成品底盘上**不是可选项**。

---

## 4. 阶段 P2：双向二进制协议（仍不接 A7A）

目标：把 P1 的文本调试口换成仓库既有的 Mick 二进制帧，并实现看门狗。**这一层做对，P3 才有意义。**

### 4.1 ESP32 侧要实现的五件事

1. **收帧**：状态机扫描 `0xAE 0xEA` → 读 `len` → 收满 `2+1+len+2` 字节 → 校验尾 `0xEF 0xFE` → 校验 `sum8`。
2. **解 `0xF3`**：三个 int16 大端除以 1000 得到 m/s、m/s、rad/s；第 7 字节记成 `last_cmd_seq`。
3. **差速逆解**：
   ```
   v_left  = linear_x - angular_z * wheel_separation / 2
   v_right = linear_x + angular_z * wheel_separation / 2
   rpm = v / (2*pi*wheel_radius) * 60
   ```
   台架建议 `wheel_radius = 0.08 m`、`wheel_separation = 0.45 m`（也可用 0.30m，见 §1）。

   > ⚠️ **上面两个数只是散件组装路线的占位示例，必须换成 §3.4 的实测值。**
   > 成品底盘通常是 **Ø65 轮 + 轮距 0.10–0.20 m**——照抄 0.08 m 会让 `odom` 距离
   > 只有实际的 40%，车冲过站点接近一倍。**填实测值，不要填文档里的数。**
4. **发 `0xA1`**（20ms 一次）：
   - x/y/yaw 由编码器积分得到（mm、mrad）；
   - **yaw 编码前必须归一化**：`yaw = atan2(sin(yaw), cos(yaw))`，否则转满 5 圈后
     int16（±32.767 rad）绕回，角度跳 2π 的整数倍，`path_tracker` 会算出反向误差、**车直接掉头**。
   - vx 取左右轮均值，wz 由左右轮差速反算；
   - battery 从 ADC1 读分压后按 §2.4 的标定系数换算，×100 发 uint16；
   - **第 12 字节 flags**（推荐发，缺省不发也能跑）：
     `bit0=fault`、`bit1=uart_timeout`、`bit2=estop_input`、`bit3=over_current`；
   - **第 13 字节 last_cmd_seq**：把最近一次收到的 `0xF3` 的 seq 原样回显。
5. **发 `0xA2`**（状态帧）：`payload[0]` 0=正常 / 非 0=fault，`payload[1..2]` 电压 ×100。

> **flags 与 fault 位不是“锦上添花”**：主机侧已经把它们接进安全链了。
> `flags` 任一位命中，或 `0xA2 payload[0]` 非 0，都会让 chassis_driver 把链路判为不可信、
> 心跳下沉为 false，500ms 后安全门零速。**所以固件不要在正常状态下误置这些位**，
> 否则车会被自己“安全停住”。调试阶段建议先只发 12 字节的 `0xA1`（不含 flags），
> 等 P5 再启用。

同时实现 **MCU 看门狗**：连续 200ms 没收到合法 `0xF3` → 目标速度置 0，**走 PID 斜坡停车**；
再 50ms 仍未收敛 → 硬断 PWM + 拉高制动，`0xA2` 的 `payload[0]` 置 1。

> **斜坡而不是硬切**：小台架无所谓，但"超时先减速、未收敛再断电"这句话在面试里是有分量的，
> 而且能避免 0.45m 轮距的车在 200ms 内从 0.5m/s 硬停导致的打滑。

### 4.2 用 PC 先验证协议（不用 A7A）

这一步能省掉大量来回烧录的时间。镜像已装 `python3-serial`（T7），容器内或宿主机都能跑；
下面按宿主机写，容器内把 `python3` 换成容器里的即可。

```bash
pip install pyserial
# 在接 ESP32 USB 口的机器上
python3 tools/bench_serial_probe.py --port /dev/ttyUSB0 --baud 460800 --mode listen
```

期望每 20ms 打印一帧解析结果（type、x/y/yaw、vx/wz、电压、sum8 是否通过）。然后再试发送：

```bash
python3 tools/bench_serial_probe.py --port /dev/ttyUSB0 --baud 460800 --mode drive --speed 0.15 --yaw 0.0 --seconds 3
```

车轮应当以约 0.15 m/s 前进 3 秒后停下。

### 4.3 拔线测试（必须做）

保持 `--mode drive` 运行，中途把 **A7A↔ESP32 的 TX/RX 线**拔掉再插上（不是拔 USB 供电线，
那样会把 MCU 一起断电，测的就变成"断电"而不是"断链"）：

- 拔掉后 **200ms 内轮子必须停**（MCU 看门狗生效）
- 插回后恢复响应，不需要重启 ESP32

测的时候把串口监视器的时间戳打开，直接读出"最后一帧合法命令"到"PWM 归零"的间隔——
**这个数字就是 F1 的实测值，不要用秒表。**

### 4.4 P2 验收

- [ ] `listen` 模式能稳定解出 `0xA1`，无 CRC 错误
- [ ] `drive` 模式下 `linear_x=0.15` 时实测速度误差 <15%
- [ ] 拔线 **200ms 内**零速，串口时间戳可读
- [ ] 故意发送错误 sum8 的帧，被丢弃且不影响后续正常帧
- [ ] `0xA2` 的 fault 位在 UART 超时时置 1，插回后复位
- [ ] **转满 6 圈以上，yaw 不出现 2π 跳变**（验证归一化）

---

## 5. 阶段 P3：接入 A7A 与 ROS 2

目标：让仓库现有的 `chassis_driver_node` 认到真底盘，`/odom` 和 `/battery_state` 出真实数据。

### 5.1 启动参数（两侧必须一致）

**首选**：不改任何 yaml，用环境变量传入（T5/T6/T4 均已落地）：

```bash
docker run -d --name amr_dispatcher \
  --network host \
  --device /dev/ttyAS4:/dev/ttyAS4 \
  -e AMR_USE_MOCK_CHASSIS=0 \
  -e AMR_SERIAL_DEVICE=/dev/ttyAS4 \
  -e AMR_CHASSIS_PROTOCOL=mick_binary \
  -e AMR_CHASSIS_BAUD=460800 \
  -e AMR_ODOM_ORIGIN_X=0 -e AMR_ODOM_ORIGIN_Y=0 -e AMR_ODOM_ORIGIN_YAW=0 \
  -e AMR_GOAL_TOLERANCE=0.3 \
  -v /home/radxa/amr_dispatcher:/workspace \
  amr-dispatcher:jazzy-arm64
```

**等价的 yaml 写法**（`config/chassis.yaml`，仅在需要固化到仓库时用）：

```yaml
chassis_driver_node:
  ros__parameters:
    serial_device: "/dev/ttyAS4"
    serial_baud: 460800          # 与 ESP32 一致
    protocol: "mick_binary"      # 关键：默认 text 会丢掉所有二进制上行帧
    poll_rate_hz: 50.0
    odom_timeout_ms: 300
    battery_full_voltage: 12.6
    battery_empty_voltage: 11.0
    cmd_stale_timeout_ms: 300    # 建议与 500ms 心跳超时错开，便于区分是哪一级先动
```

> **`AMR_CHASSIS_PROTOCOL` / `AMR_CHASSIS_BAUD` 留空即"不传"**，由 launch 默认值
> （`text` / `115200`，与 yaml 一致）决定。entrypoint 故意不抄一份默认值——
> 抄了就是第二事实源，改 launch 时两边会悄悄不一致。

> **`min_battery_voltage` 这个参数不存在**（v1 文档写错过）。`config/dispatcher.yaml` 里能用的是
> `min_battery_percent`。电量链路已在 v2.1 接通：dispatcher 订阅 `/battery_state/{vid}`，
> 换算后的百分比写进 `vehicle_states_[].battery_percent`，供 `min_battery_percent` 与
> `auto_charging_*` 使用。
>
> **但只在多车模式生效**：`/battery_state/{vid}` 是 per-vehicle 话题，只有 `vehicle_ids` 非空时才订阅。
> legacy 单车模式下 `vehicle_states_` 本来就是空的、不参与选车——所以单车台架上电量阈值
> 既不误拦也不起作用，这是设计如此，不是 bug。
>
> 另外 `battery_state_timeout_ms`（默认 3000）超时后样本作废、回落到 100：
> 底盘掉线后不能拿掉线前的最后一帧电压继续派单。详见 `HARDWARE_INTEGRATION.md` §9 T1。

### 5.2 启动（容器内）

```bash
docker run --rm -it \
  --network host \
  --device /dev/ttyAS4:/dev/ttyAS4 \
  -e AMR_USE_MOCK_CHASSIS=0 \
  -e AMR_SERIAL_DEVICE=/dev/ttyAS4 \
  -e AMR_HEALTH_RATE_HZ=5.0 \
  -v "$PWD":/workspace \
  amr-dispatcher:jazzy-arm64
```

**必须设 `AMR_USE_MOCK_CHASSIS=0`**。否则设备路径写错时会静默起 PTY 模拟器，现象是"系统一切正常、车一动不动"。

### 5.3 逐层验证（按顺序，别跳）

```bash
# ① 串口层：驱动起来了吗？
ros2 node list | grep chassis_driver
docker logs -f amr_dispatcher | grep -i "serial\|backend"

# ② 上行数据：有真实位姿吗？
ros2 topic echo /odom --once
ros2 topic echo /battery_state --once

# ③ 链路健康：link_alive 是 true 吗？
ros2 topic echo /chassis/link_health --once

# ④ 下行控制：发一个慢速指令
ros2 topic pub --rate 10 /cmd_vel_safe geometry_msgs/msg/Twist \
  "{linear: {x: 0.15}, angular: {z: 0.0}}"
```

- ② 里 `x/y/yaw` 不动 → 上行解码有问题（回 P2 用 `bench_serial_probe.py` 查）
- ③ 里 `is_healthy=false` → `odom_timeout_ms` 内没收到有效帧
- ④ 轮子不转但 ② 正常 → 检查 `protocol` 是否真的改成 `mick_binary`，以及 ESP32 是否只认文本 `CMD`

### 5.4 标定：贴标记，而不是填浮点数

真车 odom 原点一般不等于地图原点，不标定会出现"车朝反方向开"。

**推荐做法（可重复、十分钟完成）**：

1. 桌上贴一个十字标记，十字中心 = map 原点 `(0,0)`，长边指向 map **+x**。
2. `path_tracker_node` 的 `odom_origin_x/y/yaw` **保持默认 0**。
3. 上电前把车摆在十字上，车头朝 +x。
4. 节点启动后发一帧 `0xE1` 清 MCU 里程计（若固件已实现自动清零则跳过）。

这样每次掉电重启，只要"摆回标记点 + 重启节点"就恢复，**不需要重新量任何数字**。

若车确实必须停在别处，才去填参数：

```bash
# 把车摆在 map 原点、车头朝 +x，读一次 odom
ros2 topic echo /odom --once | grep -A3 pose
# 若报 x=0.12, y=-0.03, yaw=0.05，就把这三个值填进 odom_origin_*
```

### 5.5 P3 验收

- [ ] `/odom` 有非零且随轮子转动变化的 `twist`
- [ ] `/battery_state` 电压接近万用表实测值（±0.2V）
- [ ] `/chassis/link_health.is_healthy == true`
- [ ] `ros2 topic pub /cmd_vel_safe` 能让车动起来
- [ ] 摆回标记点后 `/odom` 接近 `(0, 0, 0)`

---

## 6. 阶段 P4：跑通调度闭环

目标：从 Web 下单 → 车真的开到站点 → 上报 `arrived` → 接下一单。

1. 启动完整系统（`full_system.launch.py`，容器入口默认就是它）。
2. SSH 隧道访问 Web 控制台（服务器侧只绑 `127.0.0.1:8080`）：
   ```bash
   ssh -L 8080:127.0.0.1:8080 ubuntu@81.71.76.133
   # 浏览器打开 http://127.0.0.1:8080
   ```
3. 下第一单：`station_a → station_b`，观察：
   - `dispatcher` 日志出现 `assigned`
   - `/cmd_vel_safe` 有速度输出
   - `/odom` 向目标方向移动
   - 到达后 `arrived`，任务状态终态
4. 连续下 3 单，确认队列、路权、站点占用都正常。

### P4 卡住时先查这两条

**① 车到了但不 `arrived`**：`pure_pursuit` 的 `goal_tolerance` 写死 **0.10 m**
（`pure_pursuit.hpp:36`，`path_tracker_node` 没有暴露这个参数）。标定偏几十厘米时，
车永远进不了 10cm 窗口 → 不 `arrived` → 任务挂着 → 10s 后被死锁检测判 `deadlock_detected`。
**已参数化（T4）**：启动时加 `goal_tolerance:=0.3`（或 `AMR_GOAL_TOLERANCE=0.3`），
不必再靠摆准车来凑。

**② 车不动且日志正常**：先看 `/chassis/link_health.is_healthy`。如果 `false`，
说明 `odom_timeout_ms` 内没有有效上行帧——回 P3 的 ① ② 逐层查。

### P4 验收

- [ ] 单次任务能完整闭环（不卡在某一步）
- [ ] 连续 3 单无死锁、无重复派发
- [ ] 车辆到达站点时位置误差 < 0.3m（台架手动摆位可接受）
- [ ] Web 控制台能看到实时状态

---

## 7. 阶段 P5：故障矩阵（这才是面试主战场）

每项都要记录**实测响应时间**，填进 `HARDWARE_INTEGRATION.md` §11 的表。
**测量方式决定数字可不可信**——每项都按表里的方法测，不要用秒表（人眼反应就有 200ms）。

| # | 注入手段 | 操作 | 预期 | 怎么测 |
| --- | --- | --- | --- | --- |
| F1 | 拔 UART | 车在动时拔掉 A7A↔ESP32 的 TX/RX | MCU ≤200ms 停（路径 A）；A7A 300ms 判假活（路径 B，独立证据） | **MCU 串口时间戳** |
| F2 | 杀节点 | `kill -9 chassis_driver_node` | 心跳停 → 安全门 ≤500ms 零速 | `ros2 topic echo --stamp /cmd_vel_safe` |
| F3 | 错帧注入 | `bench_serial_probe.py --mode corrupt` | 坏帧丢弃，`frames_crc_error` 递增，**不停车** | `/chassis/link_health` |
| F4 | 按急停 | 车在动时按下急停 | 物理断 12V，立刻失力 | 万用表测 VM |
| F5 | 停发指令 | 停止 pub `/cmd_vel_safe` | `cmd_stale_timeout_ms` 后驱动主动写零速（默认 500ms，本章配置改为 300ms） | 驱动日志 |
| F6 | 底盘掉电 | 关 12V 再开 | odom 归零 → **摆回标记点即恢复** | 目视 + `/odom` |
| F7 | 断 Wi-Fi | 拔网线（PC 侧断开） | 当前任务跑完停在终点，不接受新单 | 观察任务终态 |
| F8 | MCU fault | 固件置 `flags` 的 fault 位或 `0xA2 payload[0]=1` | ≤500ms 进安全门零速（T2 已实现） | `/cmd_vel_safe` |

**F1 是核心演示**：镜头同时拍到屏幕日志和轮子，拔线瞬间停转，插回自动恢复。
这段 20 秒视频比任何形容词都值钱。

**F1 的叙述方式**：不要说"三级依次触发"——三级防的是同一件事。要说
"**MCU 独立停机（不依赖 A7A）+ A7A 判假活（不依赖 MCU），两条正交路径各自成立**"。

**F7 的叙述方式**：现在"跟踪器到达后停发指令"的行为**恰好就是"完成任务后再停"**。
这是一个可以主动讲的设计选择——在窄通道里急停比跑完再停更危险。

---

## 8. 常见故障速查

| 现象 | 最可能原因 | 处理 |
| --- | --- | --- |
| `/odom` 一直是 0 | `protocol` 还是 `text` | 改 `mick_binary`，重启容器 |
| 系统正常但车不动 | 静默走了 PTY 模拟器 | 必须 `AMR_USE_MOCK_CHASSIS=0` |
| 一给指令就重启 | 12V 未共地 / 电源功率不足 | 单点共地，换 5A 电源 |
| 一接分压 ADC 就烧/读数爆表 | 用了 10k+20k（12.6V→8.4V） | 改 **100k+10k**，见 §2.4 |
| ADC 读数乱跳 | 缺 100nF / 走了 ADC2 | ADC1(GPIO32-39) + 100nF 对地 |
| 轮子反向 | 电机极性接反 | 交换 M+/M−，或软件取负 |
| 速度抖动大 | 测速窗口 1ms / 编码器干扰 | 窗口改 20ms；双绞线、10k 上拉、远离动力线 |
| 转几圈后车掉头 | yaw 未归一化，int16 绕回 | 固件 `atan2(sin,cos)` 归一 |
| 松手后速度冲过目标 | PID 只有 P/I，没有抗饱和 | 加积分限幅与输出限幅 |
| 拔线不报警 | `odom_timeout_ms` 太大 | 设为 300，且 < 500 |
| 车朝反方向开 | `odom_origin_*` 未标定 | 按 §5.4 贴十字标记 |
| 到了站点但任务不结束 | 距终点超出 `goal_tolerance`（默认 0.10m） | 设 `AMR_GOAL_TOLERANCE=0.3` 后重启 |
| 任务永远"电量不足" | 单车模式本来就不参与电量判定；多车模式下检查 `/battery_state/{vid}` 是否有数据 | 见 T1 |
| 电量百分比一直 100 | 多车模式未收到 `/battery_state/{vid}`，或样本已超 `battery_state_timeout_ms` | 见 T1 |
| MCU 报 fault 后车不动了 | **这是预期行为**：故障已进安全链（心跳下沉 → 安全门零速） | 查 `flags`/`0xA2 payload[0]` |
| `command_delivery_rate` 是 null | 正常：文本协议，或固件尚未回显 `last_cmd_seq` | 见 §4.1 第 4 条 |

---

## 9. 时间与产出

| 阶段 | 预估 | 产出 |
| --- | --- | --- |
| P0 | 2.5h | 板子可用、串口回环通过、分压标定完成 |
| P1 | 4h | 单/双电机闭环，20ms 窗口测速，PID 收敛 |
| P2 | 4h | 二进制协议 + 双向看门狗 + yaw 归一，PC 侧可验证 |
| P3 | 3h | ROS 2 拿到真实 odom/电压，标记点标定完成 |
| P4 | 4h | 调度闭环跑通 |
| P5 | 4h | 8 项故障矩阵实测表 + 演示视频 |

最终沉淀三样东西：**故障矩阵表**、**20 秒演示视频**、**README 里的硬件段落**。
这三样直接支撑简历里"底盘通信容灾与分层故障注入验证"这句话。

---

## 10. v2 相对 v1 的变更

| 项 | v1 | v2 | 后果（如果不改） |
| --- | --- | --- | --- |
| 分压电阻 | 10k + 20k | **100k + 10k** | 12.6V 输出 8.4V，**烧 ADC** |
| 减速比 | 1:30 | **1:60** | 空载 3 m/s，巡航只用 16% 转速，低速控制差 |
| 底板尺寸 | 10×15cm | **50×20cm**（或轮距缩到 0.30m） | 轮距 0.45m 的车放不下 |
| ESP32 供电 | 未写 | **USB 供电（接 PC）** | 接 A7A 的 3.3V 会因 WiFi 峰值电流复位 |
| 测速窗口 | 1kHz 里算 rpm | **1kHz PWM + 20ms 测速** | 1ms 窗口只有 2.6 个计数，误差 ~38% |
| 测速公式 | 漏减速比 | **`PPR*4*gear`** | 速度差几十倍，PID 完全跑飞 |
| yaw | 未提 | **必须归一化到 [-π,π]** | 转满 5 圈后角度跳变，车掉头 |
| 看门狗动作 | 立即断 PWM | **先斜坡，50ms 未收敛再硬断** | 0.45m 轮距硬停会打滑 |
| 拔线测试方法 | "拔 USB 线" | **拔 TX/RX 信号线** | 拔 USB 会同时断电，测的是断电不是断链 |
| F1 测量 | 秒表 | **MCU 串口时间戳** | 秒表测不出 200ms 的差异 |
| 标定 | 每次填三个浮点数 | **贴十字标记 + 上电清 odom** | 每次掉电都要重新量，F6 无法重复 |
| 故障矩阵 | 6 项 | **8 项**（+F7 断网、+F8 MCU fault） | 漏掉两个真实会发生的故障 |
| 电量 | "要改 min_battery_voltage" | **该参数不存在**；链路已在 v2.1 接通（仅多车模式） | 按不存在的参数排查，白查半天 |
| 协议 | 无序号/无故障位 | **`seq` + `flags` + `fault` 已接入安全链** | 底盘故障只打日志、车照跑 |

### v3 新增（2026-11-07：成品底盘路线）

**这一版没有改动任何协议、引脚、命令**——只增加了一条采购路线，以及随之而来的标定要求。

| 项 | v2 | v3 | 原因（不改会怎样） |
| --- | --- | --- | --- |
| 采购清单 | 单一清单（164 元） | **拆成共同项 + 路线 A / 路线 B** | 成品底盘把机械件换成整机，其余照买 |
| 轮径前提 | 全文隐含 Ø160 | **明确标出，并补 Ø65 的对照值** | Ø65 配 1:60 时 0.5 m/s 要 86% 占空比，跑不到目标速度 |
| `wheel_radius` 示例 | 直接建议 `0.08 m` | **标注为占位示例，必须换成实测值** | 照抄会让 odom 距离只有实际的 40%，车冲过站点近一倍 |
| 参数标定 | 无独立步骤 | **新增 §3.4 四项实测**（轮径/轮距/PPR/相序）+ 五步顺序 | 成品底盘省装配不省标定，填错不报错、只到不了站 |
| 编码器供电 | 「接 3.3V，不要接 5V」 | **先确认是否支持 3.3V**，否则加 74HC4050 | 5V 输出直连 GPIO34/35 会打坏 ESP32 |
| 动力线 | 只列杜邦线 | **补 18AWG 硅胶线 + 5A 自恢复保险丝** | `12V+ → 急停 → VM` 走 3–5A，杜邦线 1A 额定会发热 |
| 电阻数量 | 100k ×2 + 10k ×2 | **100k ×1 + 10k ×5** | 双电机 4 路 A/B 各需一只上拉，只买 2 只会丢两路计数 |
| 共地 | 两处 GND 相连 | **补 PC 的 GND**（ESP32 走 USB 供电时） | 漏了表现为「收到乱码但解不出帧」，易误判成波特率 |
| 进度表 | P0–P5 | P1 验收改 **target_rpm ±5%**（不含轮径） | 用 m/s 验收时轮径填错会污染结论 |

**一句话**：路线 B 买的是时间，不是免标定。把省下的一周机械工作，换成 §3.4 的一小时实测和 P5 的故障矩阵。
