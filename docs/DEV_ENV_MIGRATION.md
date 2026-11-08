# 开发环境迁移：从服务器 → 本地虚拟机

> **用途**：本文是"换机器继续干活"的交接单。
> 读完应该能回答三个问题：(1) 仓库里多出来的 `vendor/` 是什么；
> (2) 这个项目在硬件上打算怎么干；(3) 到新机器上敲哪几条命令就能接着写。
>
> 最后核对：2026-11-08

---

## 1. 一分钟版本

```bash
# 新虚拟机（Ubuntu 24.04 / x86_64）上：

git clone -b feat/phase11-observability-topology \
    https://github.com/dd2006tqf/amr-project.git
cd amr-project

# 编译（二选一，推荐容器，见 §5）
docker build -f docker/Dockerfile.jazzy -t amr-dispatcher:jazzy .
docker run --rm -v "$PWD":/workspace -w /workspace amr-dispatcher:jazzy \
    bash -lc 'source /opt/ros/jazzy/setup.bash && colcon build --base-paths src'
```

**不需要**在虚拟机上准备的东西：`build/`、`install/`、`log/` 全部被 `.gitignore` 排除，
拉下来是干净的源码树，重新编译即可。

---

## 2. `vendor/` 下的五个仓库是干什么的

`vendor/` 是 **2026-11-08 引入的上游第三方源码**（提交 `dc86535`），
共 5 个 Git 仓库、1195 个文件、约 121 MB。**原样引入，未做任何修改。**

> ⚠️ **首先澄清一个计数**：`vendor/` 下是 **5 个仓库文件夹 + 1 个 README.md**，
> 不是 6 个仓库。README 是我写的索引文件，不是上游代码。
>
> | 目录 | 是不是仓库 |
> | --- | --- |
> | `leap_ros2/` | ✅ 仓库 |
> | `leap_ros_mcu_driver/` | ✅ 仓库 |
> | `leap_ros_ws/` | ✅ 仓库 |
> | `esp32_s3_wifi_camera/` | ✅ 仓库 |
> | `voice_assistant_client/` | ✅ 仓库 |
> | `README.md` | ❌ 我写的索引文档 |

上游组织：<https://github.com/czu963889306-dev>

### 2.1 五个仓库一览

| # | 目录 | 上游仓库 | 分支 | 一行说明 | 文件数 / 体积 |
| --- | --- | --- | --- | --- | --- |
| 1 | `leap_ros2/` | `leap_ros2` | `main` | **只有硬件资料**：原理图、BOM、嘉立创工程、3D 打印外壳 | 13 / 3.1 MB |
| 2 | `leap_ros_mcu_driver/` | `leap_ros_mcu_driver` | `main` | **ESP32-S3 下位机固件**（两个 ESP 里"主控"的那颗） | 8470 / 78 MB |
| 3 | `leap_ros_ws/` | `leap_ros_ws` | **`master`** | **上位机 ROS 2 工作空间**（11 个包） | 398 / 38 MB |
| 4 | `esp32_s3_wifi_camera/` | `esp32_s3_wifi_camera` | `main` | **图传固件**，跑在独立的第二颗 ESP32-S3 上 | 181 / 2.5 MB |
| 5 | `voice_assistant_client/` | `voice_assistant_client` | `main` | **语音助手**，纯 Python 标准库，走 HTTP | 30 / 284 KB |

> 注意第 3 个仓库的分支是 **`master`** 而不是 `main`——上游自己不一致，clone 时容易踩。

### 2.2 逐个说明

#### ① `leap_ros2/` —— 硬件资料，不产生代码

```
leap_ros2/
├── README.md          # 商品说明 + QQ 群 + 许可声明
├── LICENSE
├── leap_ros.png       # 整车照片
└── 硬件/
    ├── 1.3D打印文件/   # 外壳 STL
    ├── 2.BOM/          # 物料清单（BOM_LEAP_ROS.xlsx）
    ├── 3.嘉立创工程/    # PCB 工程源文件
    ├── 4.固件/          # 预编译固件
    └── 5.原理图/        # ★ 主控板.pdf —— 查引脚走线的唯一依据
```

**对我们的用处只有一个**：`硬件/5.原理图/主控板.pdf`。

- 确认 **AT8236 的 `ISEN` / `nFAULT` 有没有走线**（决定能否低成本补故障反馈）
- 确认 **车上有没有物理急停开关的接线位**
- 确认 **A7A 能不能固定、I²C 有没有留扩展排针**

这三个问题**只能查原理图回答**，固件源码里看不出来。

#### ② `leap_ros_mcu_driver/` —— 下位机固件，**要改的就是这个**

这是 ESP32-S3 上的 C 固件，用 ESP-IDF 编译。整车能跑全靠它。

```
main/config/          board_config.h    GPIO / ADC / 外设周期
                      motion_config.h   ★ 轮径 / 轮距 / 编码器 / PID / 限幅
                      network_config.h  Wi-Fi / 串口 / 端口 / 波特率
                      ros_config.h      话题 / 服务 / frame / 发布周期
main/control/         differential_kinematics.*  两轮差速正逆解
                      motion_controller.*        ★ PID + 里程计 + 死区
main/tasks/control/   motion_task.cpp           ★ 指令超时缺失的地方（G1）
main/tasks/sensors/   lidar / imu / ultrasonic / battery
main/tasks/network/   microros/  mavlink/  provision/
components/           14 个自有组件（motor_driver / encoder_driver / camsense_lidar / ...）
managed_components/   第三方：micro_ros_espidf_component（69 MB，含预编译 libmicroros.a）
doc/protocols.md      上游自己写的 MAVLink / micro-ROS 接口说明
```

**必须知道的三个坑**（读代码核实过，不是文档里的说法）：

| # | 坑 | 位置 | 后果 |
| --- | --- | --- | --- |
| G1 | **没有指令超时** | `motion_task.cpp:50-63` | **拔线后车保持最后速度一直跑** |
| G2 | **`emergency_stop` 是死代码** | 全仓库 4 处置 `false`，**0 处置 `true`** | 急停分支 `motion_task.cpp:83` 永不命中 |
| G3 | **驱动器零故障反馈** | `components/motor_driver/motor_driver.h` | AT8236 的 `ISEN`/`nFAULT` 未接，过流/堵转看不见 |

此外还有一个环境坑：**`sdkconfig` 不在仓库里**，首次编译必须先 `idf.py set-target esp32s3` 重新生成。

#### ③ `leap_ros_ws/` —— 上位机 ROS 2 工作空间

上游写的 ROS 2 包集合，11 个包。**只有 3 类真正相关**：

| 包 | 是否相关 | 说明 |
| --- | --- | --- |
| `chuyun_bringup` | ✅ | EKF 融合（`robot_localization`）+ URDF + web 可视化裁剪 |
| `chuyun_navigation2` | ✅ | **自研轻量导航栈**，不依赖 Nav2：扫描匹配 + A* + 局部轨迹采样 + 纯跟踪 |
| `chuyun_camera` | ✅ | 用 `cv2.VideoCapture("http://<ip>:81/")` 把图传拉成 `/camera/image_raw/compressed` |
| `chuyun_wifi_config` | ⭕ | 订阅 `/wifi_config` 调 `nmcli` 配网 |
| `chuyun_description` | ⭕ | URDF + STL（29 MB，其中两个 STL 占 26 MB） |
| `chuyun_opencv` | ❌ | OpenCV 教学 demo，README 明写 **ROS 2 Humble** |
| `chuyun_cartographer` | ❌ | Cartographer 配置，依赖重、本机未装 |
| `slam_gmapping` / `openslam_gmapping` | ❌ | 老 gmapping，ROS 1 时代算法 |
| `ros2_astra_camera` | ❌ | Orbbec Astra 深度相机驱动，本项目没有该硬件 |
| `micro-ROS-Agent` / `micro_ros_msgs` | ❌ | 接自研二进制协议后用不到 |

> **⚠️ 一个真实的兼容性陷阱**：`chuyun_navigation2/param/nav2.yaml` 是**旧版 Nav2 格式**
> （`default_bt_xml_filename`、`nav2_navfn_planner/NavfnPlanner`、`recoveries_server`），
> 这些键在 **Jazzy 上已改名或移除**，直接喂给 Jazzy 的 Nav2 会启动失败。
> 真正能用的是那套自研栈（`lite_navigation.launch.py` + `config/lite_navigation*.yaml`）。

#### ④ `esp32_s3_wifi_camera/` —— 图传固件

**独立的一颗 ESP32-S3**，不跑 ROS，只做 MJPEG 图传 + 配网页。

- 与主控通过 **I²C（从机地址 `0x42`）** 交换状态和 Wi-Fi 配置
- 配网页：`http://192.168.1.88/`
- 视频流：`http://<设备IP>:81/`
- 主控可用命令：`STATUS` / `GET_IP` / `IP?` / `WIFI:ssid,password`

> **两颗 ESP32-S3 不要搞混**：主控刷 `leap_ros_mcu_driver`，图传板刷这个。
> **刷错板子就废了**，症状是"图传没了但车还能动"或反过来。

#### ⑤ `voice_assistant_client/` —— 语音助手

纯 Python 标准库（无第三方依赖），需要 ALSA 的 `arecord`/`aplay`。

流程：「你好小云」唤醒 → 阿里云百炼 ASR/LLM/TTS → 通过 stdio MCP 调用配套
**ros2-web 的 HTTP 接口**下发导航点。

**关键事实：它不直接说 ROS 2，只认 HTTP。** 所以它和本项目的 `amr_dispatcher`
之间没有现成接口，要接得先在我们的 REST gateway 上开一个导航点接口。

> 原计划是跑在 Radxa Cubie A7A 上（官方配的是 RK 板，我们用自己的 A7A 代替）。
> 技术上可行，但它依赖阿里云 API Key 与外网——**演示现场断网就废**，不能作主链路。

### 2.3 刻意没有引入的仓库

- `leap_ros_max_mcu_driver` —— 麦克纳姆轮 **MAX 配置**（约 600 元）的固件。
  本项目买的是 **LEAP_ROS 标准配置（388 元图传版）**，两轮差速，用不上。

> 仅记一个备用事实：MAX 版电池是 **3S（9.0 V 空 / 12.6 V 满）**，
> 标准版是 **2S（6.0 V 空 / 8.4 V 满）**，**两版参数不可混用**。

### 2.4 许可约束（重要）

`vendor/leap_ros2/README.md` 明确写着：

> **未经允许禁止商用，本项目仅供学习、交流和研究使用。**

所以：这批代码**只能用于学习与二次开发比对，不要用于商业分发**。
若要公开发布本项目，`vendor/` 应保持只读引用、不得作为自己的代码宣称。

### 2.5 ⚠️ `vendor/` 里有上游的硬编码 Wi-Fi 密码

**本项目仓库在 GitHub 上是 public**，而 `vendor/` 原样引入了上游代码，其中包含：

| 文件 | 内容 | 性质 |
| --- | --- | --- |
| `leap_ros_mcu_driver/main/config/network_config.h:21-22` | `kDefaultStaSsid = "Maturo"` / `kDefaultStaPassword = "maturo2026"` | 上游作者自己的测试网络 |
| `esp32_s3_wifi_camera/set_wifi.py:20` | `DEFAULT_WIFI_PASSWORD = "qwer1234"` | 上游默认值 |

**这三点要说清楚**：

1. 这些**不是我们的凭据**，是上游开源仓库本来就公开的默认值；
2. 但它们确实会**随本仓库再次公开传播**，且 `"maturo2026"` 看起来像真实网络；
3. 我们已经决定**照常提交**（2026-11-08 决策），理由是上游本就公开、且这些是测试默认值。

> **如果以后要把本项目当作品集展示给别人**：至少应在 README 或本文里声明
> \"`vendor/` 为第三方上游源码，其中含其默认测试凭据，与本项目无关\"。
> **绝不要把我们自己的真实 Wi-Fi / API Key 写进 `vendor/` 以外的任何入库文件。**

---

## 3. 硬件接入思路（我的方案）

### 3.1 一句话

**买一台 LEAP_ROS 开源小车当物理底座，把我自己写的上位机 ROS 2 调度栈接上去，
用"自研可靠协议 + 多层故障注入"作为核心卖点。**

### 3.2 三层架构

```
┌─────────────────────────────────────────────┐
│  Radxa Cubie A7A（自己加装，替代官方的 RK 板） │
│  跑 ROS 2 Jazzy 容器                          │
│  · amr_dispatcher_core   调度/规划/安全（零 ROS 依赖）│
│  · amr_dispatcher_ros    chassis_driver / safety_gate / path_tracker │
│  · REST gateway + Web 控制台                  │
└──────────────────┬──────────────────────────┘
                   │ ① USB 串口（推荐）或 ② UART 或 ③ WiFi
                   │ 协议：Mick 二进制帧 0xAE 0xEA ...
┌──────────────────▼──────────────────────────┐
│  ESP32-S3 主控（车自带，刷上游固件 + 我们补丁） │
│  · 两轮差速正逆解 / PID / 里程计               │
│  · PCNT 硬件 4 倍频读编码器                    │
│  · 雷达 UART1 / IMU / 超声 / 电池 ADC          │
└──────┬───────────────────────────┬──────────┘
       │ PWM + DIR                 │ A/B 相
┌──────▼────────┐          ┌───────▼────────┐
│ AT8236 ×2     │          │ TT 编码器电机 ×2 │
└──────┬────────┘          └────────────────┘
       │ 动力
┌──────▼──────────────────────────────────────┐
│ 2S 锂电池 ── [急停常闭] ── 电机               │
│ ＋ 独立的第二颗 ESP32-S3 做 MJPEG 图传        │
└─────────────────────────────────────────────┘
```

### 3.3 为什么买整车而不是散件

| 路线 | 机械 | 固件 | 预算 | 结论 |
| --- | --- | --- | --- | --- |
| A. 散件组装 | 自己打孔调平 | 从零写 | ≈165 元 | 时间全花在拧螺丝和调 ADC |
| B. 通用成品底盘 | 出厂预装 | 从零写 | ≈180 元 | 想练固件但没参考实现 |
| **C. LEAP_ROS** | **整机成品** | **有开源固件，改比写快** | **388 元** | **✅ 当前选定** |

**选 C 的理由**：它一次性买断了机械装配、驱动器选型、编码器/雷达/IMU 驱动代码。
时间应该花在**故障注入验证**上——那才是简历里"底盘通信容灾与分层故障注入验证"
这句话的唯一支撑。

### 3.4 核心卖点：四条正交的停机路径

**这是整个方案最重要的工程判断。** 不是"三级串联兜底"（那三级防的是同一件事），
而是**四组互不依赖的停机理由**：

| 路径 | 触发方 | 保护对象 | 目标时间 |
| --- | --- | --- | --- |
| **A. 底盘失联** | MCU 自己（200 ms 无合法下行帧） | A7A 死机时车不能继续跑 | ≤200 ms |
| **B. 上游失联** | `safety_gate`（心跳 500 ms） | 调度/跟踪节点死掉时车不能继续跑 | ≤500 ms |
| **C. 底盘故障** | MCU 上报 fault 位 | 驱动器异常时不能带病运行 | ≤500 ms |
| **D. 人身安全** | 急停开关物理断电池 | 一切软件失效 | 立即，零计算 |

**A 与 B 必须各自独立成立**——这是"拔线演示"能成立的基础：
MCU 那 200 ms 不依赖 A7A，A7A 那 500 ms 不依赖 MCU。

### 3.5 ⚠️ 但现状是：路线 C 只有路径 B 和 D

**这是必须自己补的部分，也是本文最重要的一段。**

上游固件看起来有安全逻辑，实际上三条都不成立：

| 路径 | 上游固件实际状态 | 依据 |
| --- | --- | --- |
| A. 底盘失联 | **❌ 不存在**。非阻塞取指令，取不到就沿用上一帧 | `motion_task.cpp:50-63` |
| B. 上游失联 | ✅ 成立（这是上位机侧能力，与底盘固件无关） | `safety.yaml` + `safety_gate` |
| C. 底盘故障 | **❌ 无数据来源**。AT8236 的 ISEN/nFAULT 未接 | `motor_driver.h` |
| D. 人身安全 | ⚠️ **需确认车上有没有物理急停开关** | 查原理图 |

**最危险的是路径 A 的缺失**，因为它绕过整条安全门：

| 故障 | 安全门能发现吗 | 结果 |
| --- | --- | --- |
| A7A 自己崩溃 | — | ✅ 心跳停 → 500 ms 零速 |
| **通信线被拔掉** | **❌ 发现不了** | ❌ `/cmd_vel` 仍在正常刷新，A7A 以为一切正常，**底盘却在按最后一帧跑** |

安全门保护的是"上游不发了"，**保护不了"发了但对方没收到"**。
只有 MCU 侧的看门狗能覆盖这个场景。

> **所以：在补上 G1（指令超时）之前，不要把这台车放到地上跑。**

### 3.6 第二个必须先解决的问题：通信层不兼容

固件同一时刻只启用四种模式之一：`micro_ros`（WiFi UDP，默认）、
`microros_uart`（UART0 @921600）、`mavlink_udp`（14550）、`uart_mavlink`（UART0 @230400）。

**没有一种是 Mick 二进制裸字节流。** 三条路：

| 方案 | 做法 | 代价 | 评价 |
| --- | --- | --- | --- |
| **① 加第五种模式 `mick_uart`** | 加一个分支，新建 `mick_task` 直接收发 `0xAE 0xEA` 帧 | ~300 行，协议解析可照抄 `chassis_packet.cpp` | **✅ 推荐**，改动隔离在新增文件 |
| ② 桥接 | 保留 micro-ROS，另写 `micro_ros ↔ mick_binary` 网关 | 多一个进程 + Agent 常驻 | 能跑，但多一层故障面 |
| ③ 用 MAVLink | 改走 MAVLink | 需在 `ChassisBackend` 新增编解码 | 最重；为飞控设计的协议，对差速底盘是杀鸡用牛刀 |

**上位机侧完全不用改**：`serial_backend.cpp:113` 与 `udp_backend.cpp:59`
早就有 `protocol == "mick_binary"` 分支。缺的是**下位机固件**。

### 3.7 要改的配置（只有 4 个值）

```yaml
# config/chassis.yaml
protocol: "mick_binary"        # 原 "text"
serial_baud: 921600            # 与固件 mick_uart 一致
battery_full_voltage: 8.4      # 原 12.6（2S，不是 3S）
battery_empty_voltage: 6.0     # 原 11.0
```

另有两处不在 `chassis.yaml` 里但必须改：

| 位置 | 现值 | 应改为 | 原因 |
| --- | --- | --- | --- |
| `launch/full_system.launch.py:430` 与 `:670` | `target_speed: 0.5` | **`0.25`** | 固件位置模式上限 250 mm/s |
| `config/safety.yaml` | `max_linear_x_mps: 1.5` | **`0.3`** | 安全门不该放行物理上做不到的速度 |

### 3.8 落地顺序（路线 C 专用）

| 阶段 | 目标 | 验收标准 |
| --- | --- | --- |
| P0 | 能编译、能烧录 | `idf.py set-target esp32s3` 后 build 通过 |
| **P0.5** | **⚠️ 补 G1 指令超时** | **拔线，车在 200 ms 内停**。**没做完不许落地跑** |
| P0.6 | 补 G2 急停置位源 + 死区前馈 | BOOT 键能触发零速 |
| P1 | 验证上游运动学 | `target_rpm=60` 实测 ±5% |
| P2 | 新增 `mick_uart` 模式 | A7A 与 ESP32 收发帧，`/odom` 有数据 |
| P3 | 接入 ROS（改 4 个配置值） | `/odom` 真实位姿，`/battery_state` 显示 2S 电压 |
| P3.5 | 验证上游参数 + 测低速偏差 | 0.05–0.25 m/s 五档实际/命令速度比记录在案 |
| P4 | 调度闭环 | Web 下单 → 车开到站点 → `arrived` → 接下一单 |
| P5 | 故障矩阵 | 8 项全部填实测响应时间 |

### 3.9 口径纪律（写简历/答辩时必须守）

1. **不要把设计值写成实测结果**。本文凡标 `(设计值)` 的都必须 P5 阶段填实测。
2. **不要因为车上有雷达就宣称实现了 SLAM**。车上确实有 LD-STL-14 雷达，
   但我们的调度栈**只消费 `/odom` 与 `/cmd_vel`**，没有任何节点订阅 `/scan`。
   前者是买来的，后者要自己接。
3. **不要写"四重安全保护"**。补完 G1/G2/G3 之后才能这么讲；
   在此之前准确说法是**"上位机侧安全门 + 硬件急停"**。
4. **不要抄上游固件里的标定值当成自己的实测值**。
   `motion_config.h` 里的 65 mm / 131.7 mm / 4680 是上游作者在他那台车上测的。
   正确做法：当初始值，验证并记录偏差。**这个偏差本身就是有价值的工程数据。**
5. **不要照抄商品页的 1:48 减速比**。固件写的是 4680 counts/圈（≈1:90 等效）。
   按 1:48 算，`odom` 距离会只有实际的 53%，车会跑到两倍远才停。

---

## 4. 迁移前做完了什么

| 项 | 状态 |
| --- | --- |
| `vendor/` 5 个仓库引入并提交 | ✅ `dc86535` |
| `vendor/README.md` 索引文档 | ✅ 含上游 URL、分支、许可、六处规格冲突 |
| `HARDWARE_INTEGRATION.md` v4 重写 | ✅ 新增 §3.2c / §4.0 / §10.2 / §12.1c / §15.4 |
| 固件三个缺口定位到行号 | ✅ G1/G2/G3 |
| 上位机改动量评估 | ✅ 只有 4 个配置值，C++ 一行不改 |
| **代码尚未修改** | ⏸ 按用户要求，本轮只做文档与代码归档 |

---

## 5. 新虚拟机上怎么干活

### 5.1 环境要求

| 项 | 要求 | 说明 |
| --- | --- | --- |
| 系统 | **Ubuntu 24.04 (noble) x86_64** | ROS 2 Jazzy 的目标平台 |
| Docker | 任意近期版本 | **推荐全程用容器**，见 §5.3 |
| 内存 | **≥ 8 GB**（推荐 16 GB） | 原服务器只有 3.6 GB，必须 `-j1` 串行构建；虚拟机配置够就不用受限 |
| 磁盘 | ≥ 20 GB 可用 | 源码 1.2 GB + 编译产物约 1 GB + Docker 镜像 |

> **如果直接装 ROS 2 而非用容器**：需要 `ros-jazzy-ros-base` 及
> `ros-jazzy-robot-localization`、`ros-jazzy-behaviortree-cpp`、
> `python3-serial`、`libprotobuf-dev`、`protobuf-compiler` 等。**不建议**，
> 依赖版本容易对不上，容器是唯一可复现的路径。

### 5.2 拉代码

```bash
git clone -b feat/phase11-observability-topology \
    https://github.com/dd2006tqf/amr-project.git
cd amr-project
```

> **分支说明**：本次推送后，`feat/phase11-observability-topology` 是**唯一包含
> `vendor/` 与 v4 硬件文档的分支**。`main` 落后 78 个提交，不要从 main 开始。

**拉下来应该有什么**（用来核对完整性）：

| 路径 | 应有内容 |
| --- | --- |
| `src/` | 8 个包（core / ros / bt / interfaces / proto / proto_bridge / proto_cli / tools） |
| `vendor/` | 5 个仓库目录 + README.md，约 121 MB |
| `config/` | 9 个 yaml + `scenarios/` |
| `docs/` | 8 份文档（含本文） |
| `HARDWARE_INTEGRATION.md` | 约 1600 行，v4 |
| `docker/Dockerfile.jazzy` | 容器构建入口 |

**不应该有**：`build/`、`install/`、`log/`、`.ccache/`（都被忽略，重新编译即可）。

### 5.3 构建与测试

```bash
# 构建镜像
docker build -f docker/Dockerfile.jazzy -t amr-dispatcher:jazzy .

# 编译（虚拟机内存够，可以放开并行）
docker run --rm -v "$PWD":/workspace -w /workspace amr-dispatcher:jazzy \
    bash -lc 'source /opt/ros/jazzy/setup.bash && colcon build --base-paths src'

# 跑测试
docker run --rm -v "$PWD":/workspace -w /workspace amr-dispatcher:jazzy \
    bash -lc 'source /opt/ros/jazzy/setup.bash && colcon test --base-paths src && colcon test-result --verbose'
```

> **不要用 `colcon build` 编译 `vendor/`**：构建脚本全部走 `--base-paths src`，
> `vendor/` 在仓库根目录，天然在范围外。这是刻意设计，不是巧合。

### 5.3b ⚠️ 迁移时**没有**提交的 4 个文件（必读）

服务器上有 4 个文件有未提交改动，它们**属于其他并行的工作，不在本次提交范围**。
服务器即将弃用，**这 4 个文件的改动不会通过 git 带到虚拟机**，需要单独处理。

| 文件 | 改动规模 | 性质 | 建议 |
| --- | --- | --- | --- |
| `docs/BENCH_BRINGUP_RUNBOOK.md` | +110 / −9 | 新增「路线 B 成品底盘」章节与 v3 修订 | **值得保留**，见下 |
| `docs/DEPLOYMENT.md` | +19 / −5 | 修正 `ros2 topic info` 的 `--no-daemon` 查询陷阱 | **值得保留**，见下 |
| `scripts/check_proto_dual_e2e.sh` | +183 / −5 | 新增第 4 条断言（lifecycle Activate 静默失败） | **值得保留**，见下 |
| `src/amr_dispatcher_ros/src/dispatcher_node.cpp` | +1 / −1 | **⚠️ 是一处故意的负面测试注入**，见下 | **不要提交** |

#### 关于那个 `NEGATIVE-TEST`

`dispatcher_node.cpp:616` 现在是：

```cpp
-  if (state_proto_pub_) state_proto_pub_->Activate();
+  // NEGATIVE-TEST: state Activate() 故意注释
```

这是**另一个会话为了验证 `check_proto_dual_e2e.sh` 第 4 条断言是否有效**而故意注入的缺陷
（注释掉一行 `Activate()`，看测试能不能抓到）。它**不是要保留的改动**。

> **正确做法**：回到虚拟机上，先把 `check_proto_dual_e2e.sh` 的新版本带走并跑一次，
> 确认它能抓到这个注入；**然后把这一行改回去**（恢复 `if (state_proto_pub_) state_proto_pub_->Activate();`），
> 再重新编译。**不要让这行注释跟着代码走。**

#### 那 3 个值得保留的改动怎么带走

它们是**其他对话的成果但还没提交**。三个选择：

```bash
# 方案 A（推荐）：在服务器上存成 patch，带到虚拟机
git diff docs/BENCH_BRINGUP_RUNBOOK.md docs/DEPLOYMENT.md \
         scripts/check_proto_dual_e2e.sh > /tmp/other-work.patch
# 把 /tmp/other-work.patch 拷到虚拟机后：
git apply /tmp/other-work.patch

# 方案 B：现在就单独提交这 3 个文件（不含 dispatcher_node.cpp）
git add docs/BENCH_BRINGUP_RUNBOOK.md docs/DEPLOYMENT.md scripts/check_proto_dual_e2e.sh
git commit -m "docs+test(proto): 成品底盘 runbook 与 proto 断言加固"

# 方案 C：不带走，到了虚拟机上重做（最差，浪费已完成的验证工作）
```

> **方案 A 最稳**：不会把别人正在做的半成品混进历史，也不丢东西。
> **绝不能** `git checkout` 掉这些改动——那是不可逆的（见 memory: 拆分工作树前必须备份 diff）。

### 5.4 迁移后第一件事

1. `colcon build` 通过 —— 证明工具链没问题
2. `colcon test` 全绿 —— 证明代码没在传输中损坏
3. 打开 `HARDWARE_INTEGRATION.md` §3.2c 读一遍 —— 确认硬件方案没记错

---

## 6. 待办与遗留

### 6.1 本次没有做的事（刻意）

| 项 | 为什么 |
| --- | --- |
| 改 `config/chassis.yaml` 的实际值 | 只是文档更新；改配置属于接线到货后的任务 |
| 补固件 G1/G2/G3 | 需要先确认硬件（查原理图），且属于下位机改动 |
| 改 `target_speed` 为 launch 参数 | 需要动 C++，不在本轮范围 |
| SLAM / AMCL 集成 | 调度栈不消费 `/scan`，加了是死代码 |
| 废弃 micro-ROS | 用户明确要求后续再议 |

### 6.2 到货后要先查的三件事

| # | 查什么 | 在哪查 | 影响 |
| --- | --- | --- | --- |
| 1 | AT8236 的 `ISEN`/`nFAULT` 是否走线 | `vendor/leap_ros2/硬件/5.原理图/主控板.pdf` | 决定 G3 能否低成本补上 |
| 2 | 有没有物理急停开关 | 同上 | 决定路径 D 是否成立 |
| 3 | 实测 counts/圈是不是 4680 | 手转输出轴 10 圈，读 `accumulated_count_` | 商品页 1:48 与固件 4680 差 1.875 倍 |

---

## 7. 凭据与私有文件（**迁移必读**）

以下内容**不在仓库里**，需要在新机器上重新配置：

| 项 | 现状 | 迁移后怎么办 |
| --- | --- | --- |
| **GitHub 认证** | 服务器 `~/.git-credentials` 存的是 **OAuth token（`gho_...`）** | **不会跟随迁移**。新机器上用 SSH key 或 `gh auth login` 重新配置 |
| SSH 私钥 | 只有 `id_board`（连开发板用的） | 需要为 GitHub 新建一对 key 并加到账号 |
| `机器人项目简历知识库.md` | **被 `.gitignore` 排除**（`*简历知识库.md`） | 如果还要用，**必须手动拷过去**，`git clone` 拿不到 |
| `*.pdf` | 被忽略 | 同上，手动拷 |
| 阿里云百炼 API Key | 语音助手用，当前未接入 | 若将来要接，单独管理，不要提交 |

> ⚠️ **`git clone` 拿不到的东西**：个人简历知识库、PDF、任何被 `.gitignore`
> 排除的本地文件。**迁移前先在服务器上 `ls` 一遍确认哪些要手动带走。**

---

## 8. 相关文档索引

| 文档 | 内容 |
| --- | --- |
| `vendor/README.md` | 上游五个仓库的详细索引、六处规格冲突、许可 |
| `HARDWARE_INTEGRATION.md` | 硬件方案全文（v4，约 1600 行），§3.2c 是路线 C 核心 |
| `docs/BENCH_BRINGUP_RUNBOOK.md` | 台架调试步骤 |
| `docs/DEPLOYMENT.md` | 部署说明 |
| `docs/ARCHITECTURE.md` | 软件架构 |
| `docs/DESIGN_DECISIONS.md` | 设计决策记录 |
| `README.md` | 项目总览与能力清单 |
