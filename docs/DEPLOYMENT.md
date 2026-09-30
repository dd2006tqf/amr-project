# 云服务器部署指南

将 AMR Dispatcher 完整系统(ROS 2 Jazzy,7 个运行时节点)部署到云服务器的完整 runbook。

---

## 1. 目标环境与实测基线

| 项 | 值 |
|---|---|
| 操作系统 | Ubuntu 24.04 LTS (noble) |
| 架构 | x86_64 / amd64 |
| CPU | 4 核 |
| 内存 | 3.6 GiB(可用约 1.4 GiB) |
| 磁盘 | 40 G(可用约 17 G) |
| Docker | 29.2.1 + Compose v5.0.2 |
| 用户 | `ubuntu`,已在 `docker` 组(免 sudo 可用 docker) |

### 关键约束：服务器内存不足以舒适地编译

可用内存约 1.4 GiB,而编译整套 ROS 2 C++ 工作区峰值需求更高。**因此镜像在开发机构建,服务器只负责运行。** 这是本方案与"服务器上 docker build"路线的核心区别。

---

## 2. 架构：为什么这样设计

```
开发机 (有足够内存)                      云服务器 (内存紧张)
┌────────────────────────┐              ┌──────────────────────────┐
│ docker build           │              │ docker load              │
│  ros:jazzy-ros-base    │   压缩管道    │  amr-dispatcher:jazzy    │
│  → amr-dispatcher:jazzy│ ───ssh────▶  │   (1.68GB)              │
│    1.68GB              │              │                          │
└────────────────────────┘              │  bridge 网络             │
                                        │  127.0.0.1:8080:8080     │
                                        │      ▲                   │
                                        │      │ 仅回环，公网不可达 │
                                        └──────┼───────────────────┘
                                               │
                                     SSH 隧道   │
                                        ┌──────┴───────┐
                                        │ 开发机浏览器  │
                                        │ localhost:8080│
                                        └──────────────┘
```

### 三个关键设计决策

**① 用 bridge 网络 + 回环绑定,而不是 `network_mode: host`**

`rest_gateway.cpp:106` 硬编码绑定 `INADDR_ANY`(即 0.0.0.0),且没有 bind 地址参数,应用本身也**没有任何认证机制**。在 host 网络模式下,`ports:` 映射无效,8080 会直接暴露到公网。

改用 bridge 网络 + `"127.0.0.1:8080:8080"` 后,**即使云安全组配错,8080 也只在服务器本机可达**。这是纵深防御:网络层隔离 + 只回环绑定。

因为所有节点跑在同一个容器内,DDS 通信不跨容器,bridge 网络完全够用。

**② 源码通过绑定挂载提供,不烘焙进镜像**

`rest_gateway` 和 `dispatcher_visualizer_node` 用**相对路径**加载静态资源(`tools/operator_console.html`、`config/stations.yaml`)。工作目录不对会导致 Web 控制台 404、可视化节点回退到 4 个硬编码默认站点。

入口脚本固定 `cd /workspace` 并配合 compose 的 `working_dir: /workspace` 解决这个问题。副作用是好的:**改代码只需 `git pull` + 容器内重建,不需要重新传镜像**。

**③ 用 PTY 模拟器提供底盘数据**

去掉了 `privileged: true` 和 `-v /dev:/dev`(服务器上没有 `/dev/ttyUSB0`)。入口脚本在检测不到串口设备时,自动启动项目自带的 `scripts/mock_physical_chassis.py`,解析出它创建的 `/dev/pts/N` 作为 `serial_device` 传给 launch。

这样走的是**真实 serial 后端路径**——50Hz ODOM 发布、TF 广播、`/chassis/link_health` 链路质量监控全部真实工作,比直接使用 mock 后端更接近真实运行。

> ⚠️ 若不启动模拟器,底盘不会自动降级到 mock 后端。原因:`BackendDegradationController::SwitchTo()` (`backend_degradation.cpp`) 在 `Open()` 失败时 `current_.reset()` 并 `return false`,**不更新 `current_index_`**;而 `EvaluateAndMaybeSwitch()` 要求先累积 8 帧才可能切换,`ReadPollLoop()` 在 `!IsOpen()` 时直接 return —— 形成"打不开 → 没帧 → 永不降级"的死锁。
> 好消息:心跳是**无条件发布**的(`chassis_driver_node.cpp:178`),所以看门狗不会因此熔断,系统其余部分正常运行。

---

## 3. 前置条件

### 3.1 开发机

- Docker(能拉取 `ros:jazzy-ros-base`)
- 到服务器的 SSH 免密登录
- 约 6GB 临时空闲磁盘(基础镜像 1.3GB + 新镜像 1.68GB + gzip 中转)

### 3.2 服务器

- Docker + Compose 插件
- 当前用户在 `docker` 组
- 能访问 GitHub(实测可达)
- 可用磁盘 ≥ 8GB

### 3.3 安全组 / 防火墙

**确保 8080 未对公网放行。** 服务器使用云服务商的 `YJ-FIREWALL` 链(非 ufw),默认只放行 4000/5353。若曾手动放行 8080,请移除:

```bash
# 在云控制台的安全组里删除 8080 入站规则；服务器本机可用：
sudo iptables -L INPUT -n --line-numbers | head
```

---

## 4. 首次部署

### 步骤 1：配置远程仓库访问(服务器)

仓库是公开的,HTTPS 匿名只读即可拉取。若需在服务器上 **push**,用 deploy key 或 PAT:

```bash
# 只读部署密钥
ssh-keygen -t ed25519 -f ~/.ssh/id_ed25519_amr -N ""
cat ~/.ssh/id_ed25519_amr.pub
# 把公钥加到 GitHub → 仓库 Settings → Deploy keys
# 需要 push 则勾选 "Allow write access"
```

> 本项目的部署脚本用 HTTPS 拉取(实测服务器可匿名访问),无需凭据。若仓库后来转为私有,改用 SSH 地址并在 `~/.ssh/config` 配置该密钥。

### 步骤 2：构建并传输镜像(开发机)

```bash
cd /path/to/amr_dispatcher

# 确认基础镜像就位
docker pull ros:jazzy-ros-base

# 构建 + 传输 + 远端拉起(一条命令)
AMR_SERVER=ubuntu@<server-ip> ./scripts/deploy_to_server.sh
```

该脚本会依次完成:

1. 前置检查(SSH 免密、远端 docker、基础镜像)
2. `docker build -f docker/Dockerfile.jazzy -t amr-dispatcher:jazzy .`
3. `docker save | gzip | ssh "gunzip | docker load"` —— **压缩管道直传,服务器不留中转 tar**(省约 2.5GB)
4. `tar` 同步代码到 `/opt/amr_dispatcher`(排除 `build/` `install/` `log/` 与 3MB 简历 PDF)
5. 配置远端 git 身份与 `safe.directory`
6. 调用 `scripts/server_setup.sh`:建 swap → 保证镜像存在 → `compose up -d`

### 步骤 3：等待首次构建完成

容器首次启动会在**容器内**编译整个工作区(串行度 2):

```bash
AMR_SERVER=ubuntu@<server-ip>
ssh $AMR_SERVER 'docker logs -f amr_dispatcher'
```

看到 `[amr-entrypoint] 构建完成` 与 `启动完整系统` 后即就绪。

**实测耗时(4 核 / `AMR_COLCON_WORKERS=2`,容器内首跳全量构建):**

| 阶段 | 耗时 |
|---|---|
| 阶段 1:`amr_dispatcher_core` + `amr_dispatcher_interfaces` | **11 分 36 秒** |
| 阶段 2:`amr_dispatcher_bt` + `amr_dispatcher_ros` + `amr_dispatcher_tools` | **10 分 51 秒** |
| **合计** | **约 22 分钟** |

构建期容器内存峰值仅约 270MB(远低于 1.4GB 可用),时间瓶颈在 CPU 而非内存。后续增量重建通常 1–3 分钟。

### 步骤 4：访问 Web 控制台

```bash
AMR_SERVER=ubuntu@<server-ip> ./scripts/tunnel.sh
# 另开一个终端,或直接在浏览器打开:
#   http://127.0.0.1:8080/
```

---

## 5. 验证清单

按层验证。**注意第 5 条的预期行为,不要误判为失败。**

```bash
C="docker exec -it amr_dispatcher bash -lc"
```

| # | 检查项 | 命令 | 期望 |
|---|---|---|---|
| 1 | 镜像可用 | `docker images \| grep amr-dispatcher:jazzy` | 1.68GB |
| 2 | 5 个包 | `$C 'ros2 pkg list \| grep -c amr_dispatcher'` | `5` |
| 3 | L1 核心链路 | `$C 'cd /workspace && ./scripts/check_all.sh'` | `🎉 [ALL CHECKS PASSED]` |
| 4 | 节点清单 | `$C 'source install/setup.bash && ros2 node list'` | 7 个节点(见下)。**首次查询可能少 1–2 个,是 DDS 发现抖动,重查一次即可** |
| 5 | 12 个 v2 服务 | `$C 'source install/setup.bash && ros2 service list \| grep -c /v2/'` | `12` |
| 6 | 就绪探针 | `$C 'source install/setup.bash && ros2 service list \| grep -E "system/(ready\|healthy)"'` | 2 个 |
| 7 | 生命周期 | `$C 'source install/setup.bash && ros2 lifecycle get /dispatcher_node'` | `active [3]` |
| 8 | 底盘链路 | `$C 'source install/setup.bash && ros2 topic echo /chassis/link_health --once'` | `backend_name: "serial"`,`tier: HEALTHY` |
| 9 | ODOM 有数据 | `$C 'source install/setup.bash && ros2 topic echo /odom --once'` | 有 `pose`/`twist` 数据 |
| 10 | 安全门 | `$C 'source install/setup.bash && ros2 topic info /safety/state --verbose'` | 有 1 个发布者 `safety_gate_node`。**空闲时 `echo` 无输出,见下方说明** |
| 11 | Web 控制台 | 建隧道后 `curl -s -o /dev/null -w '%{http_code}' http://127.0.0.1:8080/` | **200** |
| 12 | 快照 API | `curl -s -o /dev/null -w '%{http_code}' http://127.0.0.1:8080/api/operator/snapshot` | **200** |
| 13 | 端口未泄露 | 在服务器上 `curl -m 3 http://<公网IP>:8080/` | 超时/拒绝 |
| 14 | 重启自恢复 | `docker compose -f docker/docker-compose.server.yml restart` 后重跑 4–7 | 全部通过 |

第 4 条的 7 个节点:

```
/dispatcher_node
/safety_gate_node
/chassis_driver_node
/path_tracker_node
/rest_gateway_node
/dispatcher_visualizer_node
/lifecycle_coordinator_node
```

> **几点不要误判为失败**
>
> - **第 5 条不是笔误** —— 仓库 README 与 `test_report_matrix.json` 宣称的 `ExecuteMission.action` **实际上从未被创建**:`dispatcher_node.cpp` 里没有任何 `rclcpp_action::create_server` 调用(头文件有声明,无实例化)。所以 `ros2 action list` 为空是**当前代码的正确行为**。
> - **第 10 条 `/safety/state` 空闲时确实没有输出。** 该话题只在 `RawCmdVelCallback` 里发布(`safety_gate_node.cpp:212` 是唯一 publish 点),即**必须收到 `/cmd_vel_raw` 才会发布**。没有运动指令时静默是符合设计的反应式行为,不代表安全门坏了。要验证它工作,需先给一条速度指令:
>   ```bash
>   $C 'source install/setup.bash && ros2 topic pub --once /cmd_vel_raw geometry_msgs/msg/Twist "{linear: {x: 0.3}}"'
>   $C 'source install/setup.bash && timeout 5 ros2 topic echo /safety/state --once'
>   ```
> - **`/system/ready` 会返回 `ready=False`**,即使调度器已 `active`。这是既有缺陷,见第 9 节。

---

## 6. 日常运维

### 常用命令

```bash
cd /opt/amr_dispatcher
C="docker compose -f docker/docker-compose.server.yml"

$C ps                    # 状态
$C logs -f               # 实时日志
$C logs --tail 200       # 最近 200 行
$C restart               # 重启
$C stop / start          # 停止 / 启动
$C down                  # 停止并移除容器(镜像和代码保留)
```

### 交互式进入容器

```bash
docker exec -it amr_dispatcher bash -lc 'source install/setup.bash && bash'
```

### 快速健康巡检

```bash
docker exec -it amr_dispatcher bash -lc '
source install/setup.bash
echo "--- 节点 ---";    ros2 node list
echo "--- v2 服务数 ---"; ros2 service list | grep -c /v2/
echo "--- 底盘 ---";    timeout 5 ros2 topic echo /chassis/link_health --once
echo "--- 安全 ---";    timeout 5 ros2 topic echo /safety/state --once
'
```

---

## 7. 代码更新流程

因为源码是**绑定挂载**的,大部分更新不需要重新传镜像。

### 情况 A：改了 C++ 源码 / launch / config

```bash
# 1. 开发机推送到 GitHub
git add -A && git commit -m "..." && git push

# 2. 服务器拉取
ssh ubuntu@<server-ip> 'cd /opt/amr_dispatcher && git pull'

# 3. 容器内重新构建(仅增量,通常 1–3 分钟)
ssh ubuntu@<server-ip> 'docker exec amr_dispatcher bash -lc "
  cd /workspace && source /opt/ros/jazzy/setup.bash &&
  colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 2"'

# 4. 重启
ssh ubuntu@<server-ip> 'docker restart amr_dispatcher'
```

> 若只是改 `config/*.yaml`,`docker restart` 即可,无需重建。

### 情况 B：新增了 ROS 依赖 / 改了 Dockerfile

需要重新构建镜像并传输:

```bash
AMR_SERVER=ubuntu@<server-ip> ./scripts/deploy_to_server.sh
```

加 `--no-build` 可跳过本地构建,复用已有镜像(仅同步代码)。

---

## 8. 排障

### Web 控制台返回 404

**原因**:容器工作目录不对,`rest_gateway` 的相对路径回退失效。

```bash
# 确认容器工作目录
docker exec amr_dispatcher pwd          # 应为 /workspace
docker exec amr_dispatcher ls -l tools/operator_console.html   # 应存在
```

若 `/workspace/tools/operator_console.html` 不存在,说明代码同步不完整。`rest_gateway.cpp:280-313` 有四级回退:

1. 参数 `html_path`(默认 `tools/operator_console.html`,相对 CWD)
2. 相对路径 `tools/operator_console.html`
3. 容器内绝对路径 `/workspace/src/amr_dispatcher_all/tools/operator_console.html`(旧布局,**本方案下不存在**)
4. `ament_index` share 目录 —— **也必定失败**,因为 `amr_dispatcher_tools/CMakeLists.txt` 从未 `install()` 该 HTML

所以**只能靠前两级**,前提就是 CWD 必须是工作区根目录。

### 8080 端口占用

```bash
ss -tlnp | grep 8080
```

若被其他服务占用,改 compose 的 `ports`(如 `"127.0.0.1:8081:8080"`),并相应调整 `scripts/tunnel.sh -p`。

### 底盘无数据 (`/odom` 无输出)

```bash
# 看入口脚本是否成功解析到 PTY
docker logs amr_dispatcher 2>&1 | grep -E "serial_device|模拟器"
```

若日志显示 `WARN 未能解析模拟器 PTY`,手动验证模拟器:

```bash
docker exec -it amr_dispatcher bash -lc 'python3 scripts/mock_physical_chassis.py' 
# 正常应打印 "物理串口模拟成功！从设备端口: /dev/pts/N"
```

### 看门狗误触发 (`/safety/state` 里 `detail_message: watchdog timeout`)

**原因**:心跳发布周期 ≥ `heartbeat_timeout_ms`。compose 已把 `AMR_HEALTH_RATE_HZ=5.0`(200ms)设为远小于默认阈值 500ms,正常情况下不会触发。

若仍出现,调大安全门阈值:

```yaml
# docker-compose.server.yml 中给 safety_gate_node 加参数
# 或直接改 launch/full_system.launch.py 的 heartbeat_timeout_ms
```

### 容器反复重启

```bash
docker logs --tail 100 amr_dispatcher
docker inspect amr_dispatcher --format '{{.State.ExitCode}} {{.State.Error}}'
```

常见原因:构建失败(内存不足)、端口冲突、挂载路径不存在。把 `AMR_COLCON_WORKERS` 降到 `1` 再试。

### 内存不足 / OOM

```bash
free -h; swapon --show
dmesg | tail -30 | grep -i -E "oom|killed"
```

确认 swap 已启用(首次部署会建 4GB 的 `/swapfile.amr`)。若服务器上还有其他占内存的容器,构建期间可临时停掉:

```bash
docker stop <其他容器>   # 构建完成后 docker start 恢复
```

### 磁盘不足

```bash
df -h /; docker system df
```

可回收项:

- 容器日志已限制为 10MB × 3
- `docker image prune` 清理悬空镜像
- **服务器上原有的 4.96GB 可回收镜像**(`weaknet-*`、`docker-fastcgi_app` 等,属其他项目):确认无用后 `docker rmi`

---

## 9. 已知限制

这些是调研中发现的既有问题,**不是部署缺陷**。记录在此避免日后误判。

| 项 | 说明 |
|---|---|
| `ExecuteMission.action` 未实现 | `dispatcher_node.cpp` 无 `create_server` 调用;头文件有声明但从未实例化。README 与测试报告中的 action 契约不可用 |
| **`/system/ready` 恒返回 `ready=False`** | 见下方详细分析。这是**既有的代码缺陷**,本方案的部署产物只应用了一个推荐的健壮性修复(见第 12 节) |
| **`/safety/state` 空闲时不发布** | 唯一 publish 点在 `RawCmdVelCallback`(`safety_gate_node.cpp:212`),必须收到 `/cmd_vel_raw` 才发布。属反应式设计,不是故障;但下游依赖该话题做健康判定的逻辑在空闲期会拿不到数据 |
| **启动时有 1 次看门狗告警** | 启动后约 0.5s 会出现一次 `heartbeat_timeout 599ms`(chassis_driver 尚未发出第一拍即被判定超时)。仅 1 次、自动恢复(`watchdog_ok_` 回到 true),不影响运行。无法通过调参消除,属首次心跳竞态 |
| **底盘无串口时不自动降级** | 见第 2 节详细分析;靠启动 PTY 模拟器绕过 |
| L2 契约测试是空壳 | `test/l2_launch_contracts/test_lifecycle_contracts.py` 只做 `time.sleep(1.0)` + `assertTrue(True)`,且**从未被任何脚本或 CI 调用**;`test_report_matrix.json` 中 `L2_node_contracts: VERIFIED` 是硬编码字符串 |
| `operator_console.html` 未被安装 | `amr_dispatcher_tools/CMakeLists.txt` 无对应 `install()`,只靠 CWD 相对路径命中 |
| `ros2_control` 链路不可用 | `launch/ros2_control.launch.py` 引用的 `diff_drive_base_controller` 在本仓库内无配置。`amr_chassis_hardware_interface.so` 插件本身会正常构建 |
| `rest_gateway` 无认证 | 无任何鉴权;靠网络层隔离(回环绑定 + 隧道),**不要在公网暴露** |
| `src/.../ros2_control_plugin/` 为空目录 | 死目录,无影响 |
| `amr_dispatcher_all` 路径硬编码 | 6 处(`rest_gateway.cpp:295`、旧 compose/Dockerfile);本方案不复用该路径布局 |

### `/system/ready` 为何恒为 false（详细）

实测 `ros2 service call /system/ready ...` 返回:

```
ready=False, healthy=True, state='INITIALIZING',
active_nodes=[], pending_nodes=['dispatcher_node'],
message='DispatcherNode is in state: unknown'
```

但直接查同一个节点却可以拿到正确状态:

```bash
ros2 service call /dispatcher_node/get_state lifecycle_msgs/srv/GetState "{}"
# → current_state=State(id=3, label='active')
```

根因在 `lifecycle_coordinator_node.cpp`:

1. `CoordinatorTick()` 第一句是 `if (!dispatcher_get_state_cli_->service_is_ready()) return;` —— 该服务在**首跳时必然还未就绪**(客户端刚创建,服务端可能尚未 configure),于是直接 return,`current_dispatcher_state_` 保持构造时的初值 `"unknown"`。
2. 后续即使服务就绪,`fut.wait_for(100ms)` 这个**服务调用**超时窗口偏紧;实测该调用在容器首跳时稳定超过 100ms,于是每次都在 `if (fut.wait_for(100ms) == std::future_status::ready)` 处落空,状态**永远不被更新**。
3. 因为状态停在 `"unknown"`,`auto_bringup_` 的提拉分支(`== "unconfigured"` / `== "inactive"`)都不匹配,协调器的自动提拉**实际上从未生效**。

**关键澄清**:`dispatcher_node` 之所以仍是 `active`,是因为 `launch/full_system.launch.py` 自己用 `EmitEvent` + `OnStateTransition` 做了一套**独立的** configure→activate 级联,**与协调器无关**。所以:

- 系统功能正常,`dispatcher_node` 会到达 `active`;
- 但 `/system/ready` 这个对外就绪探针**不可信**,任何依赖它做派单拦截的上层(WMS/REST 网关闭环)都会一直拿到 `ready=False`。

这是**既有代码缺陷**,不在本次部署改动范围内。若要修复,方向是:去掉/放宽 `service_is_ready()` 的提前 return、把 `wait_for` 窗口从 100ms 放大(如 500ms),并在未拿到状态时显式重试而非静默保持 `"unknown"`。

---

## 10. 资源预算

### 服务器

| 项 | 占用 |
|---|---|
| 镜像 | 1.68GB |
| swapfile | 4.0GB |
| 代码 + 容器内构建产物 | ~0.6GB |
| **新增合计** | **~6.3GB** |
| 原可用 | 17GB |
| **部署后可用** | **~10.7GB** ✅ |

### 开发机(临时,可回收)

| 项 | 占用 |
|---|---|
| `ros:jazzy-ros-base` 基础镜像 | 1.3GB |
| `amr-dispatcher:jazzy` | 1.68GB |
| gzip 管道(不落盘) | 0 |

### 运行时内存

8 个节点峰值 RSS 约 500MB,加上 DDS 共享内存(`shm_size: 512m`),在 1.4GB 可用内存下运行是安全的。**瓶颈只在构建期,而构建已放到开发机。**

---

## 11. 回滚

```bash
# 停止并移除容器(保留镜像与代码)
ssh ubuntu@<server-ip> 'cd /opt/amr_dispatcher && docker compose -f docker/docker-compose.server.yml down'

# 回退代码到某个提交后重启
ssh ubuntu@<server-ip> 'cd /opt/amr_dispatcher && git log --oneline -10 && git checkout <sha> && docker restart amr_dispatcher'

# 删除 swapfile(若不再需要)
ssh ubuntu@<server-ip> 'sudo swapoff /swapfile.amr && sudo rm /swapfile.amr && sudo sed -i "\|/swapfile.amr|d" /etc/fstab'
```

---

## 12. 部署侧已应用的健壮性措施

以下改动是本次为服务器部署而做的,均为**向后兼容**(不传参时行为与改动前一致,不影响本机开发)。

| 措施 | 位置 | 解决的问题 |
|---|---|---|
| `serial_device` / `health_rate_hz` 参数化 | `launch/full_system.launch.py` | 原先硬编码 `/dev/ttyUSB0` 和 `health_rate_hz: 2.0`。后者等于安全门的 `heartbeat_timeout_ms: 500`,即**心跳间隔恰好等于超时阈值**,启动时必然误触发一次看门狗。部署时传 `5.0`(200ms 周期)留出余量 |
| `--base-paths src` | `docker/entrypoint.server.sh` | 仓库根的 `CMakeLists.txt` 会让 colcon 把**整个仓库**当成一个包(`amr_dispatcher_workspace`),从而忽略 `src/` 下 5 个真实包,导致 `ros2 launch` 报 `package 'amr_dispatcher_ros' not found`。与 `scripts/container_build.sh` 的既有用法一致 |
| `python3 -u` | `docker/entrypoint.server.sh` | Python stdout 重定向到文件时变块缓冲(8KB),而启动日志仅数百字节,导致 PTY 路径解析在轮询窗口内读不到内容,底盘拿不到串口设备 |
| 入口脚本从挂载仓库读取 | `docker/Dockerfile.jazzy` 的 `CMD` | 原先 `COPY` 到 `/usr/local/bin` 并直接 `ENTRYPOINT` 那个路径,脚本一经修改就必须重建镜像,否则**容器静默使用过期版本**(此坑已实际踩到一次)。现在优先用 `/workspace/docker/entrypoint.server.sh`,镜像内副本仅作兜底 |
| bridge 网络 + 回环端口绑定 | `docker/docker-compose.server.yml` | 替代 `network_mode: host` + `privileged: true` + `-v /dev:/dev`。见第 2 节 |
| 容器日志限额 | `docker/docker-compose.server.yml` | `max-size: 10m` × 3,避免长跑占满 17GB 可用磁盘 |
| `shm_size: 512m` | `docker/docker-compose.server.yml` | Fast DDS 走共享内存,默认 64MB 偏小 |

### 建议但本次未做的修复

| 建议 | 位置 | 收益 |
|---|---|---|
| 放宽协调器状态查询超时 | `lifecycle_coordinator_node.cpp`:`fut.wait_for(100ms)` → 500ms;去掉 `service_is_ready()` 提前 return 或改为重试 | 让 `/system/ready` 真实反映状态,自动提拉生效。见第 9 节 |
| 让 `safety_gate_node` 周期发布 `/safety/state` | `safety_gate_node.cpp`:在 `WatchdogTick()`(100ms 定时器)里也调用 `PublishSafetyState` | 空闲期也有安全状态可观测,下游健康判定不再落空 |
| 启动时对受限后端做 fall-through | `backend_degradation.cpp`:`SwitchTo()` 在 `Open()` 失败时继续尝试下一个后端 | 无串口时自动落到 mock,不再依赖外部 PTY 模拟器 |
