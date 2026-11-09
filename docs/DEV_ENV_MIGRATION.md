# 开发环境迁移：从服务器 → 本地虚拟机

> **用途**：本文是"换机器继续干活"的交接单。
> 读完应该能回答四个问题：(1) 仓库里多出来的 `vendor/` 是什么；
> (2) **当前融合到哪一步了**（见 §0）；(3) 这个项目在硬件上打算怎么干；
> (4) 到新机器上敲哪几条命令就能接着写。
>
> 最后核对：2026-11-08

---

## 0. ⚠️ 当前状态：**尚未融合**

**这部分是最容易被误读的，先讲清楚。**

### 0.1 两个项目现在是完全独立的

| | `amr_dispatcher`（我们的） | `vendor/`（上游的） |
| --- | --- | --- |
| 位置 | `src/` | `vendor/` |
| 参与编译 | ✅ 是 | ❌ **否** |
| 能被 `colcon build` 扫到 | ✅ 是 | ❌ 否（`--base-paths src` 刻意排除） |
| 与对方有代码级调用 | ❌ 无 | ❌ 无 |
| 运行时会互相发现 | ❌ 不会 | ❌ 不会 |

**`vendor/` 下 4246 个文件，一行都未进入 `src/` 的构建，也没有被任何包 `find_package` 或 `#include`。**

### 0.2 为什么编译能成功

因为编译的是**原本就能独立工作的 `amr_dispatcher`**，不是"融合后的项目"。

`amr_dispatcher` 是一个完整自洽的项目：8 个包、176 个提交、1252 个测试。
它从一开始就不依赖 `vendor/`——`vendor/` 是 2026-11-08 才引入的**只读参照源码**。

> **换句话说**：`colcon build` 成功只能证明"我们的项目自身完好"，
> **不能**证明"已经能驱动真车"。

### 0.3 融合的三个层次与当前进度

| 层 | 内容 | 状态 |
| --- | --- | --- |
| **L1 上位机协议层** | `mick_binary` 编解码分支 | ✅ **早就写完了**，`serial_backend.cpp` / `udp_backend.cpp` / `chassis_packet_stream.cpp` 均已实现 |
| **L2 下位机固件** | ESP32 增加 `mick_uart` 模式 | ❌ **未开始**。固件目前只有 `micro_ros` / `microros_uart` / `mavlink_udp` / `uart_mavlink` 四种，**没有一种能吃 `0xAE 0xEA` 帧** |
| **L3 整机联调** | 改配置 + 上台架 + 验里程计 | ❌ **未开始**。依赖 L2 完成与硬件到货 |

> **关键结论**：融合的缺口不在上位机，而在**下位机固件**。
> 上位机侧"几乎不用改"是真的（只改 4 个配置值），但那不是"融合快完成了"，
> 而是"我们这边早就备好了，等固件那边接上"。

### 0.4 ⚠️ 一个比融合更紧急的前置条件

上游固件 `motion_task.cpp:50-63` **没有指令超时**：拔掉通信线后，
车会保持最后一帧的速度**一直跑**。安全门只能防"上游不发"，防不了"发了但对方没收到"。

**在补上 G1（指令超时）之前，不要把车放到地上跑。** 详见 §3.5。

### 0.5 三个容易搞混的目录

| 目录 | 里面是什么 | 有没有代码 |
| --- | --- | --- |
| `vendor/leap_ros2/` | **只有硬件资料**：原理图 PDF、BOM、嘉立创工程、3D 外壳 | ❌ **0 个代码文件**（13 个文件全是 pdf/xlsx/epro/zip） |
| `vendor/leap_ros_mcu_driver/` | **ESP32-S3 下位机固件**（真正的融合目标） | ✅ 3623 个文件 |
| `vendor/leap_ros_ws/` | **上游上位机 ROS 2 代码**（11 个包） | ✅ 398 个文件 |

> **常见误解**：以为 `leap_ros2` 是"官方上位机代码"。
> **不是**——`leap_ros2` 是硬件资料仓库。真正的上游上位机代码在 `leap_ros_ws`。
> 而 `leap_ros2` 唯一的用途是 `硬件/5.原理图/主控板.pdf`，用来查 GPIO 走线。

---

## 1. 一分钟版本

```bash
# 新虚拟机（Ubuntu 24.04 / x86_64）上：

git clone https://github.com/dd2006tqf/amr-project.git
cd amr-project/upper/amr          # ← colcon 工作区根在这一层

# 编译（二选一，推荐容器，见 §5）
docker build -f docker/Dockerfile.jazzy -t amr-dispatcher:jazzy .
docker run --rm -v "$PWD":/workspace -w /workspace amr-dispatcher:jazzy \
    bash -lc 'source /opt/ros/jazzy/setup.bash && colcon build --base-paths src'
```

> ⚠️ **2026-11-08 起仓库分为 `upper/` `lower/` `hardware/` 三层**，`vendor/` 已拆掉。
> 上位机的所有命令都要在 `upper/amr/` 下执行；在仓库根执行 `colcon build` 会发现 0 个包。
> 分层说明见 `docs/UPSTREAM_VENDOR.md`。

**不需要**在虚拟机上准备的东西：`build/`、`install/`、`log/` 全部被 `.gitignore` 排除，
拉下来是干净的源码树，重新编译即可。

---

## 2. 上游第三方源码是干什么的（原 `vendor/`，2026-11-08 已拆为三层）

这 5 个 Git 仓库、**4246 个文件、约 72 MB（入库体积）** 于 **2026-11-08 引入**（提交 `dc86535`），
**原样引入，未做任何修改**。原目录名 `vendor/` 已拆除，**名字一个没改，只改了层级**：

| 原位置 | 新位置 | 理由 |
| --- | --- | --- |
| `vendor/leap_ros_mcu_driver/` | `lower/leap_ros_mcu_driver/` | 下位机固件 |
| `vendor/esp32_s3_wifi_camera/` | `lower/esp32_s3_wifi_camera/` | 下位机图传固件 |
| `vendor/leap_ros_ws/` | `upper/leap_ros_ws/` | 上游官方上位机 |
| `vendor/voice_assistant_client/` | `upper/voice_assistant_client/` | 上位机附加应用 |
| `vendor/leap_ros2/` | `hardware/leap_ros2/` | 纯硬件资料，无代码 |
| `vendor/README.md` | `docs/UPSTREAM_VENDOR.md` | 由目录说明改为文档 |

> 📌 **三个数字不要搞混**（2026-11-08 干净克隆实测修正）：
> · **72.3 MB / 4246 文件** = 真正提交进 git 的内容（`git ls-tree` 逐文件字节求和），
>   `git clone` 拿到的就是这些；
> · **85.5 MB** = `vendor/` 工作树的真实字节数（`du -sb` = 89 704 427 B）；
> · **121 MB** = 服务器上 `du -sh vendor/` 的**块占用**。它 = 入库内容 72.3 MiB
>   + 未入库构建产物 13.2 MiB，**再加约 35 MB 的文件系统块粒度开销**（小文件多）。
>   **不要用 `du -sh` 判断入库体积**，那个数字永远偏大。
>
> 其中 `micro_ros_dev/{build,install,log}` 是 **4841 个** cmake 中间产物，
> 真实字节 **13.2 MB**（块占用约 35 MB），**刻意排除、不入库**。
> 这三处早先文档分别写错过（「1195 个文件」「4843 个」「49 MB」），以本页为准。

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
| 2 | `leap_ros_mcu_driver/` | `leap_ros_mcu_driver` | `main` | **ESP32-S3 下位机固件**（两个 ESP 里"主控"的那颗） | 3623 / 62 MB |
| 3 | `leap_ros_ws/` | `leap_ros_ws` | **`master`** | **上位机 ROS 2 工作空间**（11 个包） | 398 / 38 MB |
| 4 | `esp32_s3_wifi_camera/` | `esp32_s3_wifi_camera` | `main` | **图传固件**，跑在独立的第二颗 ESP32-S3 上 | 181 / 2.5 MB |
| 5 | `voice_assistant_client/` | `voice_assistant_client` | `main` | **语音助手**，纯 Python 标准库，走 HTTP | 30 / 284 KB |

> 注意第 3 个仓库的分支是 **`master`** 而不是 `main`——上游自己不一致，clone 时容易踩。

### 2.2 逐个说明

#### ① `leap_ros2/` —— 硬件资料，**一个代码文件都没有**

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

> ⚠️ **最容易搞混的一点**：这个仓库**不是**"官方上位机代码"。
> 全仓库 13 个文件、**0 个代码文件**（`cpp/py/h/CMakeLists/package.xml` 全都没有），
> 内容是 2 个原理图 PDF + 3 个 BOM 表格 + 2 个嘉立创工程 + 1 个外壳 zip + 照片/许可。
> **它不参与编译，也不可能参与编译。**
>
> 真正的上游上位机 ROS 2 代码在 **`leap_ros_ws/`**（见下方 ③）；
> 真正的固件代码在 **`leap_ros_mcu_driver/`**（见 ②）。

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

##### 这 11 个包与我们的项目是什么关系（2026-11-08 结论）

**先说结论：目前一个都没有用上，也一个都没有编译。** 而且它们与我们的调度栈**存在架构竞争**：

| | `amr_dispatcher`（我们的） | `chuyun_navigation2`（上游的） |
| --- | --- | --- |
| 谁规划路径 | 自己的 `SkeletonPlanner` + 行为树 | 扫描匹配 + A* + 局部轨迹采样 |
| 消费什么 | 只用 `/odom`，发 `/cmd_vel` | 要 `/scan`、要自己维护地图 |
| 安全层 | `safety_gate` + 看门狗 + 物理急停 | 无 |

> **两者是竞争关系，不是互补。** 把 `chuyun_navigation2` 整个装进来会和我们的调度栈抢
> `/cmd_vel` 和规划权。**不要直接集成。**

真正值得从 `leap_ros_ws` 拿的只有三样（**且都还没拿**）：

| 项 | 位置 | 价值 | 状态 |
| --- | --- | --- | --- |
| EKF 融合配置 | `chuyun_bringup/config/ekf.yaml` | 把编码器 odom 与 IMU 融合。我们现在只有纯编码器 odom | ❌ 未接入 |
| URDF / TF 树 | `chuyun_description` | 真车尺寸与 `base_link`→`laser` 变换 | ❌ 未接入 |
| 自定义服务 | `leap_interfaces/srv/`（只有 3 个：`SetSpeedPid` / `GetSpeedPid` / `SetArmPose`） | 调 PID 参数、机械臂姿态。**没有自定义 msg** | ⭕ 可选 |

> 注：`leap_interfaces` 里**没有**里程计/底盘状态类的 msg —— 上游走的是 micro-ROS 的
> 标准 msg（`nav_msgs/Odometry` 等），所以我们不需要它的消息定义。

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

**上位机侧代码已经就绪，不需要改**：`serial_backend.cpp:113` 与 `udp_backend.cpp:59`
早就有 `protocol == "mick_binary"` 分支。缺的是**下位机固件**的 `mick_uart` 模式。

> ⚠️ **不要把这句话读成「融合快完成了」**。准确含义是：
> 我们这边（上层）早就备好了协议实现，但**下层固件还没有对应的收发模式**，
> 两边的字节流至今从未真正通过一次。融合的实际工作量在 `vendor/leap_ros_mcu_driver/`，
> 不在本仓库的 `src/`。

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
| 上位机改动量评估 | ✅ 只有 4 个配置值，C++ 一行不改（**但这不等于融合完成**，见 §0） |
| **融合状态** | ⏸ **未融合**：`vendor/` 未参与编译，L2 固件 `mick_uart` 未开始，见 §0.3 |
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

> ⚠️ **"内存够就不用受限" 这句话在实践中反复翻车，2026-11-09 再次复现。**
> 判据不是 `nproc`，也不是 `MemTotal`，而是 **`MemAvailable` 与 swap 水位**：
>
> ```bash
> free -m    # 看 available 与 swap used，不看 total
> ```
>
> 只要 `available` < 4 GB **或** swap 已接近用满，就必须 `-j1`。
> 实测反例：本机 `MemTotal` 7.5 GB（按 `scripts/lib/build_workers.sh` 的
> "每 worker 2 GB" 公式会算出 **2**），但 `MemAvailable` 只有约 3.1 GB、
> swap 2.0/2.0 GiB 已满 —— 用 2 路并行构建 15 分钟只推进到 31%，
> 最终被超时杀掉。**`build_workers.sh` 只看 MemTotal，对本机不安全。**
>
> **另一个必须避开的坑**：`build_workers.sh` 是**可执行脚本**（输出一个整数），
> 不是 shell 函数。用 `source` 加载它只会把数字打印到屏幕，变量不会被赋值；
> 此时 `cmake --build . -j"$W"` 会退化成 `-j""`，语义是**不限并发**，
> 而不是"默认值"。传并发度前务必校验非空：
>
> ```bash
> W=$(bash scripts/lib/build_workers.sh); [ -n "$W" ] || W=1
> ```


> **如果直接装 ROS 2 而非用容器**：需要 `ros-jazzy-ros-base` 及
> `ros-jazzy-robot-localization`、`ros-jazzy-behaviortree-cpp`、
> `python3-serial`、`libprotobuf-dev`、`protobuf-compiler` 等。**不建议**，
> 依赖版本容易对不上，容器是唯一可复现的路径。

### 5.2 拉代码

```bash
git clone https://github.com/dd2006tqf/amr-project.git
cd amr-project
```

> **分支说明（2026-11-08 更新）**：所有工作已合入 **`main`**（提交 `8c1aa71`）。
> **直接 clone 默认的 `main` 即可**，不需要 `-b` 指定分支。
>
> 早期版本的本节让人 clone `feat/phase11-observability-topology`，那个分支
> 已过期（停在 `0780690`，落后 main **4** 个提交），**不要再使用**。

**拉下来应该有什么**（用来核对完整性）：

| 路径 | 应有内容 |
| --- | --- |
| `src/` | 8 个包（core / ros / bt / interfaces / proto / proto_bridge / proto_cli / tools） |
| `vendor/` | 5 个仓库目录 + README.md，**4246 个文件 / 72.3 MiB 入库内容**（`du -sh` 会显示 86 MB，那是块占用，不是入库体积） |
| `config/` | 10 个顶层 yaml + `scenarios/`（其中 2 个场景 yaml） |
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

### 5.3b 能还原到什么程度（2026-11-08 已全部入库）

早先本节列了 4 个"未提交、需要手动带走"的文件。**已全部提交并推送到 `main`**：

| 文件 | 处理结果 |
| --- | --- |
| `docs/BENCH_BRINGUP_RUNBOOK.md` | ✅ 已提交（`8c1aa71`） |
| `docs/DEPLOYMENT.md` | ✅ 已提交（`8c1aa71`） |
| `scripts/check_proto_dual_e2e.sh` | ✅ 已提交（`8c1aa71`） |
| `src/amr_dispatcher_ros/src/dispatcher_node.cpp` | ✅ **负面测试已还原**，库内是正确的 `Activate()` |

所以**不再需要手工搬运 patch**，`git clone` 就是全部源码。

#### ✅ 能 100% 还原的

| 项 | 说明 |
| --- | --- |
| 全部源码 | `src/` 8 个包，**176 个提交**的完整历史（其中 116 个触碰 `src/`） |
| `vendor/` 4246 个文件 | 含之前漏掉的 `openni2_redist`（**18 个文件，其中 9 个 `.so`**），已用 `git add -f` 入库 |
| `config/`、`launch/`、`docs/` | 全部在库 |
| 构建脚本、Docker 文件 | 全部在库 |

#### ❌ 不能还原的（**这不是缺陷，是设计**）

| 项 | 为什么 | 到虚拟机怎么办 |
| --- | --- | --- |
| `build/` `install/` `log/` | 编译产物，被 `.gitignore` 排除 | 重新 `colcon build` |
| `micro_ros_dev/{build,install,log}` 4841 个文件 / 13.2 MB | cmake 中间产物，换机器必然重新生成 | 用时重新生成 |
| `.ccache/` | 编译缓存 | 自动重建（首次编译会慢些） |
| **本地分支**（5 个 feat/refactor，另有 `main`） | git 只推 `main` | 不需要，相对 `main` 独有提交均为 0 |
| **`构建产物`**：`build/` `install/` `log/` `runtime/` | 全部被 `.gitignore` 排除 | 重新 `colcon build` / 重新生成 |
| **`~/.git-credentials` 里的 token** | 不在仓库 | 新机器上重新配认证 |

> **2026-11-08 变更**：`机器人项目简历知识库.md` 与 `docs/superpowers/`
> 已从 `.gitignore` 移出并入库（用户明确要求），**不再是手工项**。
> 仓库是 **public**，二者推上去即永久公开、无法真正撤回，这一点已确认接受。
> 仍然排除的只剩 `*.pdf`（`vendor/` 下的原理图另有 `!` 规则放行）与构建产物。

#### 结论

**代码层面：100% 可还原**（`git clone` 一条命令，含 `vendor/`、全部文档、
简历知识库与 `docs/superpowers/` —— 已用干净克隆逐 blob 校验，4830 个文件全等）。
**环境层面：需重编译**（几分钟到十几分钟，取决于机器）。
**唯一的手工项：仓库外的 `*.pdf`** —— 若你本地还留着一份，拷过去即可；
仓库内两份原理图已在库，不影响查引脚。

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
| `机器人项目简历知识库.md` | ✅ **2026-11-08 已入库** | `git clone` 即可拿到 |
| `docs/superpowers/` | ✅ **2026-11-08 已入库**（13 个文件 / 87 KB） | `git clone` 即可拿到 |
| `*.pdf`（仓库外的） | 被 `.gitignore` 排除 | 若还需要，手动拷；仓库内原理图已在库 |
| 阿里云百炼 API Key | 语音助手用，当前未接入 | 若将来要接，单独管理，不要提交 |

> ⚠️ **`git clone` 拿不到的东西**：只剩 `*.pdf`（仓库外）、构建产物与
> `~/.git-credentials` 里的 token。**迁移前先在服务器上 `ls` 一遍确认。**

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
