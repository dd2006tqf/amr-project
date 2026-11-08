# vendor/ —— 上游第三方源码（只读参照）

本目录存放 **出云 LEAP_ROS 开源小车**的上游源码，原样引入、未做任何修改。
用途只有一个：作为"自研上位机 × 厂商下位机"融合工作的**对照基准**和**接口契约来源**。

> **不要在这几个目录里写代码。** 需要改的部分一律落到我们自己的 `src/` / `config/` / `launch/`。
> 上游文件只能作为"原始事实"被引用（行号引用会随上游更新失效，引用时请连带写出内容）。

## 仓库清单

上游组织：<https://github.com/czu963889306-dev>

| 目录 | 上游仓库 | 分支 | 作用 | 文件数 | 体积 |
| --- | --- | --- | --- | --- | --- |
| `leap_ros2/` | `leap_ros2` | `main` | **仅硬件**：主控板/图传板原理图、BOM、嘉立创工程、3D 打印外壳 | 13 | 3.1 MB |
| `leap_ros_mcu_driver/` | `leap_ros_mcu_driver` | `main` | **ESP32-S3 下位机固件 v2.1**，两个 ESP 里"主控"的那颗 | 3623 | 62 MB |
| `leap_ros_ws/` | `leap_ros_ws` | **`master`** | 上位机 ROS 2 工作空间（11 个包） | 398 | 38 MB |
| `esp32_s3_wifi_camera/` | `esp32_s3_wifi_camera` | `main` | 图传固件，跑在**独立的第二颗 ESP32-S3** 上 | 181 | 2.5 MB |
| `voice_assistant_client/` | `voice_assistant_client` | `main` | 语音助手（纯 Python 标准库，走 ros2-web HTTP） | 30 | 284 KB |

引入提交：`dc86535`（2026-11-08）。**上游未打 tag，也没有 release 记录**，所以这里只能锁定到"某个时间点的 HEAD"，不是一个可复现的版本号。

## 完整性：怎么核对"原样引入"这四个字

**入库 4246 个文件 / 约 72 MB。** 工作树里 `vendor/` 占 121 MB，多出的 49 MB 全部是
`micro_ros_espidf_component/micro_ros_dev/{build,install,log}` 的 4843 个 cmake 中间产物，
**刻意排除**（换机器必然重新生成，且与 LEAP_ROS 固件无关）。

随时可以这样验证（应当输出 `OK`）：

```bash
# 1) 没有被漏掉的文件：除了构建树，不应再有未跟踪文件
git ls-files -o vendor | grep -v 'micro_ros_dev/\(build\|install\|log\)/'
# 期望：无输出

# 2) 与上游 tarball 逐文件比对（需要 /home/ubuntu/个人学习/leap_ros_vendor 原件）
for d in leap_ros_vendor/*/; do
  b=$(basename "$d")
  comm -23 <(cd "$d" && find . -type f -not -path './.git/*' | sed 's|^\./||' | sort) \
           <(cd "amr_dispatcher/vendor/$b" && find . -type f | sed 's|^\./||' | sort)
done
# 期望：无输出
```

### ⚠️ 一个真实的坑：根目录的 `!` 规则穿不透子目录 `.gitignore`

`openni2_redist/{arm,arm64,x64}/` 里 18 个 `.so`/`.ini`（4.9 MB）**曾经被漏掉**。

原因：根 `.gitignore` 里写了 `!vendor/.../openni2_redist/**`，看着像放行了，
但 `ros2_astra_camera/.gitignore:400-404` 另有 `x64/`、`[Aa][Rr][Mm]/`、`[Aa][Rr][Mm]64/`
三条**更深层**的规则。git 的规则是**最深配置文件优先**，根目录的否定规则根本穿不透。

> **教训**：在 `vendor/` 里判断某文件会不会入库，**不要靠读 `.gitignore` 推断**，
> 直接用 `git check-ignore -v <path>` 看命中的是哪一行；最终以
> `git ls-files -o --exclude-standard vendor`（未被忽略的遗漏）为准。
>
> 修复方式用了 `git add -f`（文件一旦被跟踪，忽略规则不再作用），
> 这样**不用修改上游的 `.gitignore`**，`vendor/` 与上游保持严格 1:1。
> 根 `.gitignore` 里那两条失效规则保留在原地并加了警示注释，防止后人再踩。

## 刻意未纳入

- `leap_ros_max_mcu_driver` —— 麦克纳姆轮 **MAX 配置**（约 600 元）的下位机固件。
  本项目采购的是 **LEAP_ROS 标准配置（388 元图传版）**，两轮差速，不使用该仓库。
  仅记录一个事实备用：MAX 版文档里电池是 **3S（9.0 V 空 / 12.6 V 满）**，
  与标准版的 **2S（6.0 V / 8.4 V）** 不同，**两版参数不可混用**。

## 目录速查

### `leap_ros_mcu_driver/`（要改的就是这个）

```
main/config/          board_config.h  GPIO/ADC/外设周期
                      motion_config.h 轮径/轮距/编码器/PID/限幅  ← 机械参数都在这里
                      network_config.h Wi-Fi/串口/端口/波特率
                      ros_config.h    话题/服务/frame/发布周期
main/control/         differential_kinematics.*  两轮差速正逆解
                      motion_controller.*        PID + 里程计 + 位置/相对运动
main/tasks/control/   motion_task.cpp            ← 指令超时缺失的位置，见下
main/tasks/sensors/   lidar / imu / ultrasonic / battery
main/tasks/network/   microros/  mavlink/  provision/
components/           14 个自有组件（motor_driver / encoder_driver / camsense_lidar / ...）
managed_components/   第三方：micro_ros_espidf_component（69 MB，含预编译 libmicroros.a）
components/mavlink_lib/   第三方 mavlink 头文件（13 万行，只是生成代码，不必读）
doc/protocols.md      上游自己写的 MAVLink / micro-ROS 接口说明
```

**必须知道的三个坑**（读代码时核实过，不是文档里的说法）：

1. **没有指令超时。** `main/tasks/control/motion_task.cpp:50-63` 用
   `xQueueReceive(..., 0)` 非阻塞取指令，取不到就继续沿用上一帧 `cmd_msg` 调用
   `robot.Update()`。**链路断开后车会保持最后速度继续跑**。
   方案里的"MCU 200 ms 看门狗（路径 A）"在当前固件里**不存在**，必须自己写。
2. **`emergency_stop` 是死代码。** 全仓库只有把该标志置 `false` 的 4 处
   （`microros_subscribers.cpp:17`、`gamepad_i2c_task.cpp:197`、
   `mavlink_common.cpp:779` / `:835`），**没有任何一处置 `true`**。
   所以 `motion_task.cpp:83` 那个急停分支永远不会命中。
3. **`sdkconfig` 不在仓库里**（上游 `.gitignore` 只忽略 `sdkconfig.old`，
   但我们拿到的归档里确实没有）。首次编译前必须先 `idf.py set-target esp32s3` 重新生成。

### `leap_ros_ws/`（11 个包，只有 3 类真正相关）

| 包 | 是否相关 | 说明 |
| --- | --- | --- |
| `chuyun_bringup` | ✅ | EKF 融合（`robot_localization`）+ URDF + web 可视化裁剪节点 |
| `chuyun_navigation2` | ✅ | **自研轻量导航栈**，不依赖 Nav2：扫描匹配定位 + A* + 局部轨迹采样 + 纯跟踪 |
| `chuyun_camera` | ✅ | 用 `cv2.VideoCapture("http://<ip>:81/")` 把图传拉成 `/camera/image_raw/compressed` |
| `chuyun_wifi_config` | ⭕ | 订阅 `/wifi_config` 调 `nmcli` 配网，并向固件回灌 `/host_info` |
| `chuyun_description` | ⭕ | URDF + STL（29 MB，其中 `controller.stl` 15 MB / `tt_motor.stl` 11 MB） |
| `chuyun_opencv` | ❌ | OpenCV 教学 demo（HSV 分割/形状识别），**README 明确写 ROS 2 Humble** |
| `chuyun_cartographer` | ❌ | Cartographer 配置，依赖重、本机未装 |
| `slam_gmapping` / `openslam_gmapping` | ❌ | 老 gmapping，ROS 1 时代算法 |
| `ros2_astra_camera` | ❌ | Orbbec Astra 深度相机驱动（6.8 MB），本项目没有该硬件 |
| `micro-ROS-Agent` / `micro_ros_msgs` | ❌ | micro-ROS 侧；接自研二进制协议后用不到 |

**注意 `chuyun_navigation2/param/nav2.yaml` 是旧版 Nav2 格式**：
`default_bt_xml_filename`、`nav2_navfn_planner/NavfnPlanner`、
`recoveries_server` 这些键在 **Jazzy 上已改名或移除**，直接喂给 Jazzy 的 Nav2 会启动失败。
真正可用的是 `chuyun_navigation2` 那套自研栈（`lite_navigation.launch.py` + `config/lite_navigation*.yaml`）。

### `esp32_s3_wifi_camera/`

独立的一颗 ESP32-S3，**不跑 ROS**，只做 MJPEG 图传 + 配网页，与主控通过
**I²C（从机地址 0x42）** 交换状态和 Wi-Fi 配置。主控侧对应实现见
`leap_ros_mcu_driver/components/camera_i2c_client/`。

- 配网页：`http://192.168.1.88/`
- 视频流：`http://<设备IP>:81/`
- 主控可用 I²C 命令：`STATUS` / `GET_IP` / `IP?` / `WIFI:ssid,password` 等（见 `main/i2c_slave_comm.c`）

### `voice_assistant_client/`

纯 Python 标准库（无第三方依赖），需要 ALSA 的 `arecord`/`aplay`。功能是
"你好小云"唤醒 → 阿里云百炼 ASR/LLM/TTS → 通过 stdio MCP 调用
**配套 ros2-web 的 HTTP 接口**下发导航点。

**它不直接说 ROS 2**，只认 HTTP。所以它和我们的 `amr_dispatcher` 之间没有现成接口，
要接的话需要一层 HTTP 适配（对应我们现有的 REST gateway）。

## 采购型号与代码的对应关系

本项目买的是 **LEAP_ROS 图传版（388 元）**。规格表里有多处与代码不符，**以代码为准**：

| 规格表说法 | 代码事实 | 依据 |
| --- | --- | --- |
| 减速比 1:48 | `kPulsesPerWheelRev = 4680`。13 PPR × 4 倍频 × **90** = 4680；1:48 配 13 PPR 只有 2496 | `main/config/motion_config.h:14` |
| 最大速度 0.8 m/s | 位置模式限速 **250 mm/s**；手柄上限 **300 mm/s** | `motion_config.h:28`、`gamepad_i2c_task.cpp:24` |
| 雷达 USB 直连上位机 | **UART1 @115200，RX = GPIO41**，由固件转成 `/scan` | `network_config.h:52`、`board_config.h:37` |
| 3S 电池 | 标准版是 **2S：8.4 V 满 / 6.0 V 空**，GPIO3 / ADC1_CH2 | `battery_adc.cpp:15-17` |
| ROS 2 Humble | 本项目是 **Jazzy**。`chuyun_opencv` README 明写 Humble | 上游文档 |
| "官方固件" | 个人开源项目（README 有 QQ 群，注明"未经允许禁止商用"），**不是厂商维护的产品** | `leap_ros2/README.md` |

## 许可

上游 `leap_ros2/README.md` 明确：

> **未经允许禁止商用，本项目仅供学习、交流和研究使用。**

引入本目录仅用于学习与二次开发比对，**不要用于商业分发**。
