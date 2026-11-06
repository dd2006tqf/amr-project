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
2. `docker build -f docker/Dockerfile.jazzy -t amr-dispatcher:jazzy .`(加 `--no-build` 则跳过)
3. `docker save | gzip | ssh "gunzip | docker load"` —— **压缩管道直传,服务器不留中转 tar**(省约 2.5GB)
4. 校验本地 HEAD 已推送到 `origin/main`(未推送则拒绝部署,可用 `--allow-unpushed` 跳过)
5. 调用 `scripts/server_setup.sh`:建 swap → 服务器自行 `git fetch + reset --hard origin/main` 同步代码 → 保证镜像存在 → `compose up -d`

> **为什么代码由服务器自己从 git 拉取,而不是从开发机推文件过去。**
> 早先的实现用 `tar | ssh | tar x` 同步整个仓库(含 `.git/`),后果是服务器原有的
> `.git/config` 被开发机版本覆盖,一次造成两处回归:上游跟踪丢失(服务器 `git pull`
> 报 `There is no tracking information for the current branch`)、提交身份被改成开发机的
> `tanqf`。根因是不该用文件同步去搬运 `.git/`。现在服务器保留自己的 `.git/config`
> (含上游跟踪与凭据),代码的唯一来源是远端仓库。

### 步骤 3：等待首次构建完成

容器首次启动会在**容器内**编译整个工作区(串行度 2):

```bash
AMR_SERVER=ubuntu@<server-ip>
ssh $AMR_SERVER 'docker logs -f amr_dispatcher'
```

看到 `[amr-entrypoint] 构建完成` 与 `启动完整系统` 后即就绪。

**实测耗时。** 服务器(4 核 / `AMR_COLCON_WORKERS=2`,容器内首跳全量构建):

| 阶段 | 耗时 |
|---|---|
| 阶段 1:`amr_dispatcher_core` + `amr_dispatcher_interfaces` | **2 分 11 秒** |
| 阶段 2:`amr_dispatcher_bt` + `amr_dispatcher_ros` + `amr_dispatcher_tools` | **2 分 23 秒** |
| **构建合计** | **约 4 分 35 秒** |
| 构建完成 → 7 个节点就绪 | **约 4 秒** |

作为参照,同一份代码在开发机(同时跑着其他容器)上首跳耗时约 22 分钟。构建期容器内存峰值仅约 270MB(远低于 1.8GB 可用),时间瓶颈在 CPU 而非内存。后续增量重建通常 1–3 分钟。

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
| 2 | 7 个包 | `$C 'ros2 pkg list \| grep -c amr_dispatcher'` | `7` |
| 3 | L1 核心链路 | `$C 'cd /workspace && ./scripts/check_all.sh'` | `🎉 [ALL CHECKS PASSED]` |
| 4 | 节点清单 | `$C 'source install/setup.bash && ros2 node list'` | 7 个节点(见下)。**首次查询可能少 1–2 个,是 DDS 发现抖动,重查一次即可** |
| 5 | 15 个 v2 服务 | `$C 'source install/setup.bash && ros2 service list \| grep -c /v2/'` | `15` |
| 6 | 就绪探针 | `$C 'source install/setup.bash && ros2 service list \| grep -E "system/(ready\|healthy)"'` | 2 个 |
| 7 | 生命周期 | `$C 'source install/setup.bash && ros2 lifecycle get /dispatcher_node'` | `active [3]` |
| 8 | 底盘链路 | `$C 'source install/setup.bash && ros2 topic echo /chassis/link_health --once'` | `backend_name: "serial"`。`tier` 见下方说明 |
| 9 | ODOM 有数据 | `$C 'source install/setup.bash && timeout 8 ros2 topic hz /odom'` | **约 50 Hz**(实测 49.0–49.5) |
| 10 | 安全门 | `$C 'source install/setup.bash && ros2 topic info /safety/state --verbose'` | 有 1 个发布者 `safety_gate_node`。**空闲时 `echo` 无输出,见下方说明** |
| 11 | Web 控制台 | 建隧道后 `curl -s -o /dev/null -w '%{http_code}' http://127.0.0.1:8080/` | **200** |
| 12 | 快照 API | `curl -s http://127.0.0.1:8080/api/operator/snapshot` | 返回 JSON,含 `"chassis_backend":"serial"` |
| 13 | 端口未泄露 | 在服务器上 `curl -m 8 http://<公网IP>:8080/` | `000`(连接失败) |
| 14 | 重启自恢复 | `docker compose -f docker/docker-compose.server.yml restart` 后重跑 4–7 | 全部通过 |
| 15 | 容器健康态 | `docker ps --format '{{.Status}}' \| grep amr_dispatcher` | 含 `(healthy)`（首个 180s 为 `starting`，属正常） |
| 16 | 探针免鉴权 | `curl -s -o /dev/null -w '%{http_code}' http://127.0.0.1:8080/readyz` | `200`（配了 `auth_token` 也一样） |
| 17 | 节点守护 | 容器内 `kill -9 <dispatcher_node pid>`，等 10s 再看 `ros2 lifecycle get /dispatcher_node` | `active [3]`，且 `docker exec ... pgrep -f dispatcher_node` 是**新** PID |

#### 端到端功能验证（可选但推荐）

上面的检查只证明节点起来了。要证明**指令链路真的通**,需下发速度指令:

```bash
C="docker exec amr_dispatcher bash -lc"
S='source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash'

# 1. 订阅 /safety/state 放到后台,再发指令(它只在收到指令时发布)
$C "$S && (timeout 22 ros2 topic echo /safety/state > /tmp/safety.txt 2>&1 &) ; sleep 3 ; \
     timeout 16 ros2 topic pub --rate 20 --times 20 /cmd_vel_raw geometry_msgs/msg/Twist '{linear: {x: 0.3}}'"

# 2. 安全门放行证据: severity=INFO, motion_allowed=true, speed_scale=1.0
$C "head -14 /tmp/safety.txt"

# 3. 指令确实写入了串口: frames_success 随指令数增长
$C "$S && timeout 12 ros2 topic echo /chassis/link_health --once" | grep -E 'frames_|tier|loss_rate'
```

实测结果:`/safety/state` 返回 `severity: INFO, motion_allowed: true, speed_scale: 1.0`(全速放行);
`/chassis/link_health` 的 `frames_success` 与下发指令数一致、`loss_rate: 0.0`。

> **`tier` 的预期值别搞错。** 判定逻辑在 `link_quality.cpp` 的 `RecomputeLocked()`:
> `healthy_loss_rate` 达标 **且** `consecutive_success_ >= success_threshold` 才置 `kHealthy`,
> 而 `success_threshold` 默认 **50**(`link_quality.hpp:16`)。
>
> 所以:
> - 从未下发指令 → `tier: UNKNOWN`,`frames_total: 0`(正常,`frames_total` 统计的是**已下发的指令数**,不是读到的 odom 帧)
> - 下发 1–49 条 → `tier: DEGRADED`,`is_healthy: false`(**正常预热**,不是故障)
> - 下发 ≥50 条 → `tier: HEALTHY`,`is_healthy: true`
>
> 实测:21 条指令时 `tier: DEGRADED`;发到 81 条时转为 `tier: HEALTHY, is_healthy: true`。

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
cd /home/ubuntu/amr_dispatcher
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

### 6.1 探针与自愈：HEALTHCHECK 的真实边界

先说结论，因为它决定了一切：**HEALTHCHECK 只提供可见性，不触发自动重启。**

| 机制 | 它做什么 | 它**不**做什么 |
|---|---|---|
| `docker healthcheck`（`docker/healthcheck.py` → `GET /readyz`） | `docker ps` 显示 `healthy/unhealthy`；`compose up --wait` 等待就绪 | 不重启容器；不替代应用自愈 |
| launch 的节点级 `respawn`（`respawn_enabled`） | 子进程崩溃后自动拉起（延迟 `respawn_delay` 秒） | 不做健康判定（只对"进程死了"起作用） |
| `lifecycle_coordinator_node`（`auto_bringup=true`） | 把重生后回到 `unconfigured` 的 dispatcher 重新提到 `active` | 不负责拉起进程 |

**为什么禁止接 autoheal**：`/readyz` 为 503 可能是"健康但安全锁存"（比如急停还没解除）。
因 503 重启容器会拆掉整套栈、丢掉调度队列与在途任务的路权状态，把一个
"需要人来按急停复位"的场景变成一个"反复重启 + 反复丢单"的场景。
probe 的作用是**让人知道**，不是**替人决策**。

**自愈链的依赖关系（必须记住）**：`dispatcher_node` 的 respawn 依赖
`lifecycle_coordinator_node` 存活（它负责 unconfigured→active）。
协调器自己也在 respawn 名单里，但如果**整个容器**死了，就只剩 compose 的
`restart: unless-stopped`——那是容器级恢复，代价是一次冷启动。

**冷启动的副作用（如实告知）**：respawn 是冷重启。`on_configure` 会从
`state_snapshot_file` 恢复队列并把在途任务重新入队，车辆可能收到第二份
`/plan` / VDA order。这是既有的重启语义（`check_state_persistence_e2e.sh` 锁定），
只是现在无需人工触发。

**探针分两条线**（`docs/API.md` §5.1.1）：

```bash
# liveness：只回答"网关还能处理 HTTP"，DDS 全挂时仍然是 200
docker exec amr_dispatcher curl -s localhost:8080/healthz

# readiness：逐组件聚合，任一不就绪回 503，body 里有每个组件的 reason
docker exec amr_dispatcher curl -s localhost:8080/readyz | python3 -m json.tool
```

`/readyz` 的 `reason` 就是排障入口：

| `reason` | 含义 | 先查什么 |
|---|---|---|
| `ok` | 新鲜且语义健康 | — |
| `never_seen` | 从未收到该组件的数据 | 节点起来了吗？话题名/域 ID 对吗？`state` 字段同时会是 `INITIALIZING` |
| `stale` | 收到过，但已超过 `stale_after_seconds` | 该组件进程还活着吗（`ps` / `ros2 node list`）？DDS 卡死？`detail` 里带实测时延 |
| `unhealthy` | 新鲜但语义不健康 | 看具体组件：`motion_allowed` / `is_healthy` / `ready_flag`；安全门再看 `/safety/state` |
| `not_monitored` | 该部署形态下无法观测 | 多车模式下的 `chassis`，属正常 |

`chassis` 组件在**多车模式**下会显示 `monitored: false`、`reason: not_monitored` 且
**不**计入失败——那时 `/chassis/link_health` 是 per-vehicle 话题，网关订阅不到全局话题。
这是诚实的"我在关注但拿不到数据"，不是"假装健康"。

> 字段名以 `health_probe.hpp::RenderReadinessJson` 为准，别凭印象写脚本：
> 数组键是 `components`，每项组件名键是 `component`（不是 `name`），
> liveness 里是 `uptime_seconds`（不是 `uptime_sec`）。

### 6.2 主动告警（Webhook）

默认**关闭**：`alert_webhook_url` 为空时不起巡检定时器、不起发送线程，
进程/线程/话题与关闭前完全一致（`check_health_supervision_e2e.sh` T8 断言）。

启用方式（`config/rest_gateway.yaml` 或 launch 参数）：

```yaml
rest_gateway_node:
  ros__parameters:
    alert_webhook_url: "http://alertmanager.internal:9093/api/v2/alerts"
    alert_not_ready_threshold: 3      # 连续 3 次不就绪才升级
    alert_min_interval_sec: 300       # firing 期间 5 分钟提醒一次
    alert_safety_critical_sec: 30     # CRITICAL 持续 30s 才告警
    alert_queue_capacity: 64          # 满时丢最旧
```

- **只支持 `http://`**。`https://` 启动时 `ERROR` 并保持禁用（本构建不含 TLS）。
  需要加密请在前面挂一个本地 http 中继。
- payload schema 为 `amr.alert.v1`，字段见 `include/amr_dispatcher_tools/alerting.hpp`。
- 恢复通知**不受节流限制**，且每次故障只发**恰好一条** `*_recovered`。
- 发送失败不重试，只累加 `amr_alert_webhook_failures_total` —— 让抓取侧可以对
  "告警通道本身"告警。这是刻意的：重发一条过时的告警不如让下一拍发新的。

告警规则样例见 `deploy/alerts/amr_dispatcher.rules.yml`。

### 6.3 施工区 runbook：临时封路 / 单向 / 临时标石

**不要**再进容器 `vim stations.yaml`：那既无连通性校验，也无在途车仲裁，
而且 `map_ingest` / Web 地图上传会整份覆盖该文件，手改的管制会被静默抹掉。

标准流程（单条边）：

```bash
# 0) 先看现状
curl -s localhost:8080/api/v1/topology/overlay?yaml=1 | python3 -m json.tool

# 1) 预演（可选但强烈建议）：dry_run 下绝不写盘
curl -s -X POST localhost:8080/api/v1/topology/preview -d '{
  "overlay_yaml": "blocked_edges:\n  - from: \"B3\"\n    to: \"B4\"\n    reason: \"water main\"\n",
  "operator_id": "op_zhang", "reason": "water main repair"
}'

# 2) 提交
curl -s -X POST localhost:8080/api/v1/topology/block_edge -d '{
  "from_station": "B3", "to_station": "B4", "direction": "both",
  "operator_id": "op_zhang", "reason": "water main repair",
  "expected_map_hash": "<从 GET /api/v1/topology/overlay 取>", "confirm": true
}'

# 3) 收工清空
curl -s -X POST localhost:8080/api/v1/topology/unblock_edge -d '{
  "from_station": "B3", "to_station": "B4",
  "operator_id": "op_zhang", "reason": "work complete", "confirm": true
}'
```

**409 的含义与处置**（详见 `docs/API.md` §5.5 的 `error_code` 表）：

| `error_code` | 现场怎么处理 |
|---|---|
| `would_disconnect_topology` | 这条边是桥边，封了会让某些站点互相到不了。换一条，或先加临时绕行边。 |
| `edge_occupied_in_flight` | 有车可能正压在这条边上。**等它驶离**再提交；确需立即生效且能保证车辆驶离，用 `allow_edge_under_vehicle=true`（该边锁留给在途车）。 |
| `would_strand_vehicles` | 有在途任务找不到替代路线。先让那些任务完成/取消，或先开绕行。 |
| `map_hash_mismatch` | 客户端手里的地图版本过期了。重新 `GET /api/v1/topology/overlay` 取新 hash。 |
| `lock_conflict` | 新路线要用的路权被别的任务占着，稍后重试。 |

**文件生命周期**（都在 `topology_overlay_file` 同目录）：

| 文件 | 来源 | 处置 |
|---|---|---|
| `overlay.yaml` | 提交成功时原子写入（tmp → fsync → 回读校验 → rename） | 正常生效中。**不要手改**：手改后等下一个热重载周期，但少了校验与在途仲裁。 |
| `overlay.yaml.bak` | 每次提交前把旧文件改名 | 上一版。手工回滚 = `mv overlay.yaml.bak overlay.yaml` 后重启 dispatcher。 |
| `overlay.yaml.stale.<epoch>` | 基础地图 `map_hash` 变化时自动归档 | 叠加层已失效，文件留作审计。确认无用后可删。 |
| （无 `corrupt` 后缀） | 文件损坏时**不**改名、**不**动原文件 | 进程会 `ERROR` + 审计 `topology_overlay_rejected`，并**按基线目录运行**。看日志查原因，修好或删掉文件再重启。 |

**功能开关与默认值**：容器入口脚本默认注入
`/workspace/config/generated/topology_overlay.yaml`（可用
`AMR_TOPOLOGY_OVERLAY_FILE` 覆盖，设为空串 = 关闭）。它与 `stations.yaml`
**同目录但不同文件**——地图上传会整份覆盖 `stations.yaml`，叠加层必须独立
持久化才不会被抹掉。直接 `ros2 run` 时 `topology_overlay_file` 默认为空 =
功能关闭，此时写操作返回 `501 overlay_disabled`（读接口仍可用，`message`
会写明 disabled）。**这是刻意的**：没有持久化就没有原子性，绝不能让施工区
"内存里生效、重启就没了"。

**提交顺序即安全铁律**（`CommitTopologyPlan`）：落盘 → 预约新锁 → 释放废弃锁 →
为被封边加 `traffic_block:` 锁 → 解封已移除的边 → 换内存叠加层 → 同步规划器 → 下发改道。
进程在落盘之后任意时刻死掉，重启从冷启动回放就是目标态；死在落盘之前，
哪里都没变。因此**磁盘写失败时内存零改动**，不需要"回滚内存锁"这种最容易写错的分支。

---

## 7. 代码更新流程

因为源码是**绑定挂载**的,大部分更新不需要重新传镜像。

### 情况 A：改了 C++ 源码 / launch / config

```bash
# 1. 开发机推送到 GitHub
git add -A && git commit -m "..." && git push

# 2. 服务器拉取
ssh ubuntu@<server-ip> 'cd /home/ubuntu/amr_dispatcher && git pull'

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

> ⚠️ **重建后必须重启容器,改动才生效。** `colcon build` 只更新磁盘上的二进制,
> 运行中的进程仍是旧版本。实测:重建单个包约 **29 秒**,重启到系统就绪约 **1 秒**。
> 在重启之前,系统处于"部分节点已更新、部分仍是旧版"的混合状态。
>
> ```bash
> ssh ubuntu@<server-ip> 'cd /home/ubuntu/amr_dispatcher && \
>   docker exec amr_dispatcher bash -lc "cd /workspace && source /opt/ros/jazzy/setup.bash && \
>     colcon build --base-paths src --symlink-install --packages-select <包名> \
>       --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release" && \
>   docker compose -f docker/docker-compose.server.yml restart'
> ```

---

## 7.1 在服务器上直接开发与提交

服务器已具备完整的读写能力，可以直接改代码、提交并推送到远端。

### 当前状态（实测）

| 能力 | 状态 |
|---|---|
| 完整运行项目 | ✅ 7 个业务节点 / 15 个 `/v2/` 服务 / `dispatcher_node` active |
| 改代码并增量重建 | ✅ 单包约 29 秒；重启约 1 秒 |
| `git commit` | ✅ 身份 `dd2006tqf <dd2006tqf@users.noreply.github.com>` |
| `git clone` / `pull` | ✅ |
| **`git push`** | ✅ 已配置,写权限经临时分支实测确认 |

### 凭据配置方式（HTTPS + credential store）

服务器用 HTTPS 方式，凭据存在 `~/.git-credentials`（权限 `600`），
`credential.helper = store`：

```
https://dd2006tqf:***@github.com
```

该 token 由开发机的 `~/.git-credentials` 同步而来（仅同步 `github.com` 那一条，
未同步开发机上另一条代理条目）。

**维护要点：**

- 该 token 现在同时存在于**开发机与服务器**两处。若在 GitHub 上轮换或吊销，
  两台机器都要同步更新，否则服务器推送会开始报认证失败。
- 凭据是 GitHub **Personal Access Token**（不是账号密码），需要 `repo` 权限。
- 若怀疑泄露，在 GitHub → Settings → Developer settings → Personal access tokens
  中吊销即可，两台机器重新配置。

### 替代方案：SSH Deploy Key

若希望避免 token 落盘，可改用 Deploy Key：

```bash
ssh ubuntu@<server-ip> 'ssh-keygen -t ed25519 -f ~/.ssh/id_ed25519_amr -N "" && cat ~/.ssh/id_ed25519_amr.pub'
```

把公钥加到 GitHub 仓库 → **Settings → Deploy keys**，**务必勾选 "Allow write access"**，然后：

```bash
ssh ubuntu@<server-ip> 'cd /home/ubuntu/amr_dispatcher && \
  git remote set-url origin git@github.com:dd2006tqf/amr-project.git && \
  ssh -T git@github.com && git push --dry-run origin main'
```

注意：Deploy Key **只能绑定单个仓库**，且同一把公钥不能同时作为账号级 SSH key 使用。

### 推荐的工作方式：就在 `/home/ubuntu/amr_dispatcher` 里开发，养成"先推送"的习惯

容器挂载的就是 `/home/ubuntu/amr_dispatcher`，直接在这里改代码最省事 —— 改完增量构建 +
重启即可，不需要额外的工作副本。

**要记住的一点：跑部署脚本前先把改动推到 `origin/main`。**

#### `git reset --hard` 会丢什么（实测结论）

`server_setup.sh` 每次都会把工作区强制同步到 `origin/main`。实测确认：

| 内容 | 是否会被丢弃 |
|---|---|
| 未提交的工作区改动 | **会丢** |
| 已 `commit` 但**未 `push`** 的提交 | **会丢** —— 分支指针被移回远端位置，你的提交从分支上消失 |
| 已 `push` 的提交 | **不会丢** —— 它们已在 `origin/main` 上，reset 的目标就是它们 |

所以「只要 commit 过就安全」**是错的**：`reset --hard origin/main` 是按指针移动，
不是按内容比对，未推送的提交照样会离开分支。**`git push` 才是真正的保护。**

万一忘了推也不要紧：被 reset 掉的提交通常还能从 `git reflog` 找回（未被 gc 前）。
实测 `git reflog` 能看到 `reset: moving to origin/main` 之前的 `commit` 记录，
用其哈希 `git checkout <hash>` 即可恢复。

#### 已有保护性拦截：不会静默覆盖

`server_setup.sh` 现在会检查未推送的提交与未提交的改动，**一发现就中止**并列出内容：

```
[FAIL] /home/ubuntu/amr_dispatcher 有未同步到 origin/main 的本地内容，已中止以免丢失：

        未推送的提交: 2 个
        未提交的改动: 1 个

        git reset --hard 会丢弃这两类内容。请先处理：

          先推送（推荐）:
            cd /home/ubuntu/amr_dispatcher && git add -A && git commit -m "..." && git push origin main

          或先备份到别处:
            cd /home/ubuntu/amr_dispatcher && git stash push -u -m "部署前备份"

          确认要放弃这些改动（危险，只能靠 git reflog 找回）:
            AMR_FORCE_SYNC=1 ./scripts/server_setup.sh
```

同一份检查也在 `deploy_to_server.sh` 里（校验本地 HEAD 是否为 `origin/main` 的祖先）。

#### 完整开发流程

```bash
S=ubuntu@<server-ip>

# 1. 进入容器改代码（容器内 /workspace 就是宿主机的 /home/ubuntu/amr_dispatcher）
ssh $S 'docker exec -it amr_dispatcher bash -lc "cd /workspace && source /opt/ros/jazzy/setup.bash && bash"'

# 2. 增量构建（单包实测约 29 秒）
ssh $S 'docker exec amr_dispatcher bash -lc "cd /workspace && source /opt/ros/jazzy/setup.bash && \
  colcon build --base-paths src --symlink-install --packages-select <包名> \
    --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release"'

# 3. 重启使新构建生效（实测约 1 秒到就绪）
ssh $S 'cd /home/ubuntu/amr_dispatcher && docker compose -f docker/docker-compose.server.yml restart'

# 4. 验证改动真的生效（看日志/话题，不要只看构建成功）
ssh $S 'docker logs amr_dispatcher 2>&1 | tail -20'

# 5. 提交并推送 —— 这一步是防丢的关键
ssh $S 'cd /home/ubuntu/amr_dispatcher && git add -A && git commit -m "..." && git push origin main'
```

> **第 2 步之后必须第 3 步。** `colcon build` 只更新磁盘上的二进制，运行中的进程
> 仍是旧版本。重启之前系统处于"部分节点已更新、部分仍是旧版"的混合状态。
> 已实测：在容器内改掉 `safety_gate_node` 的一条启动日志、构建成功后，容器日志里
> 仍是旧文案；重启后才出现新文案。

全程耗时参考：改一行代码 → 构建 29 秒 → 重启到就绪 1 秒，**约半分钟一轮**。

#### 如果你更想分离开发与部署副本

也可以另建一个副本（例如 `~/dev/amr_dispatcher`），在那边改代码、提交、推送，
再让 `/home/ubuntu/amr_dispatcher` 拉取。好处是两边互不干扰，代价是多一次同步。

`/home/ubuntu/amr_dispatcher` 下的 `build/` `install/` `log/` 均已被 `.gitignore` 忽略，
所以直接在里面开发时 `git status` 不会被构建产物污染。

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
docker inspect amr_dispatcher --format 'ExitCode={{.State.ExitCode}} RestartCount={{.RestartCount}} OOMKilled={{.State.OOMKilled}}'
```

常见原因:构建失败(内存不足)、端口冲突、挂载路径不存在。把 `AMR_COLCON_WORKERS` 降到 `1` 再试。

**若日志里是 `package 'amr_dispatcher_ros' not found`,先看这一条。** 这是实际遇到过的
失败模式(容器重启 13 次,每次都在同一处):

```
ament_cmake_symlink_install_directory() can't find
  '<workspace>/src/amr_dispatcher_tools/include/'
Failed   <<< amr_dispatcher_tools
Aborted  <<< amr_dispatcher_ros
```

根因是 git 不跟踪空目录 —— `src/amr_dispatcher_tools/include/` 下没有任何被跟踪文件,
全新 clone 后目录不存在,而 `CMakeLists.txt` 要用 `install(DIRECTORY include/ ...)`。
仓库已用 `.gitkeep` 修复。**若你遇到此错,说明服务器上的代码早于该修复:**

```bash
ssh ubuntu@<server-ip> 'cd /home/ubuntu/amr_dispatcher && git pull --ff-only && sudo rm -rf build install log'
# 然后重启容器
ssh ubuntu@<server-ip> 'cd /home/ubuntu/amr_dispatcher && docker compose -f docker/docker-compose.server.yml restart'
```

判断构建是否真的成功,看这一行(它统计的是 `install/` 下的包目录,不受日志干扰):

```
[amr-entrypoint] 构建完成，install/ 下已安装 5 个项目包
```

**看到 `3 个` 或更少就是失败了** —— 正常必须是 5 个(`core` / `interfaces` / `bt` / `ros` / `tools`)。

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

## 10. 资源预算（实测）

### 服务器

| 项 | 占用 |
|---|---|
| 镜像 `amr-dispatcher:jazzy` | 1.7GB |
| swapfile | 4.0GB |
| 仓库 + 容器内构建产物(`/home/ubuntu/amr_dispatcher`) | 159MB |
| **新增合计** | **约 5.9GB** |
| 部署前可用 | 17GB |
| **部署后可用** | **11GB** ✅ |

实测磁盘变化:部署前 `22G 已用 / 17G 可用` → 部署后 `28G 已用 / 11G 可用`。

### 开发机(临时,可回收)

| 项 | 占用 |
|---|---|
| `ros:jazzy-ros-base` 基础镜像 | 1.3GB |
| `amr-dispatcher:jazzy` | 1.68GB |
| gzip 管道(管道直传,不落盘) | 0 |

镜像传输实测:1.7GB 镜像经 `docker save \| gzip -1 \| ssh` 管道,**95 秒**完成(含远端 `docker load`)。

### 运行时内存

实测容器 **287MB RSS / CPU 4.7%**(7 个节点 + PTY 模拟器,空闲态)。加上 DDS 共享内存
(`shm_size: 512m`),在 1.8GB 可用内存下运行安全。

**瓶颈只在构建期**:实测构建峰值内存约 270MB(4 核并行度 2),远低于可用内存,未触发 OOM
(`OOMKilled=false`)。容器日志已限额(`max-size: 10m` × `max-file: 3`),长期运行不会撑爆磁盘。

> 注意:服务器 swap 总用量在部署后从 1.7GiB 升至约 2.6GiB。其中大部分来自服务器上**原有的其他服务**
> (vault / keycloak / temporal / minio / pgvector 等),本项目的容器只占 287MB。

---

## 11. 回滚

```bash
# 停止并移除容器(保留镜像与代码)
ssh ubuntu@<server-ip> 'cd /home/ubuntu/amr_dispatcher && docker compose -f docker/docker-compose.server.yml down'

# 回退代码到某个提交后重启
ssh ubuntu@<server-ip> 'cd /home/ubuntu/amr_dispatcher && git log --oneline -10 && git checkout <sha> && docker restart amr_dispatcher'

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
| 补 `include/amr_dispatcher_tools/.gitkeep` | `src/amr_dispatcher_tools/include/amr_dispatcher_tools/` | **全新 clone 无法构建**:该目录下没有任何被跟踪文件,而 git 不跟踪空目录,于是全新 clone 后目录不存在,`amr_dispatcher_tools/CMakeLists.txt` 的 `install(DIRECTORY include/ ...)` 直接失败(`ament_cmake_symlink_install_directory() can't find ...`),并连带中止 `amr_dispatcher_ros`。最终 `install/` 缺两个包,`ros2 launch` 报 `package 'amr_dispatcher_ros' not found`。**该问题只在全新 clone 时暴露** —— 开发机上这个目录一直存在(历史遗留的本地目录),所以本地 cmake/colcon 构建与全部 453 个测试全程通过。服务器实测时容器因此反复重启 13 次,每次都在同一处失败 |
| 显式 `name: amr-dispatcher` | `docker/docker-compose.server.yml` | compose 默认以所在目录名作项目名,本文件在 `docker/` 下故项目名为 `docker`。实测该服务器上 `tc_fcgi_app` / `tc_fcgi_nginx_fastdfs` / `tc_fcgi_mysql` 三个长期运行的容器**同属 `docker` 项目**,启动本栈时已出现 orphan containers 警告。一旦对同名项目执行 `--remove-orphans`,那三个无关容器会被一并删除 |
| 代码改由服务器从 git 同步 | `scripts/deploy_to_server.sh`、`scripts/server_setup.sh` | 原先用 `tar \| ssh \| tar x` 推代码(含 `.git/`),会把服务器的 `.git/config` 覆盖成开发机版本,一次造成两处回归:上游跟踪丢失、提交身份被改成 `tanqf`。现改为服务器 `git fetch + reset --hard origin/main`,`.git/config` 逐字节不变(已用 md5 前后比对确认) |
| 排除 `__pycache__` / `*.pyc` | `.dockerignore`、`scripts/server_setup.sh` | 容器以 root 运行,Python 启动 launch 文件时会在挂载的仓库里写出 root 所有的 `.pyc`。这些文件既污染 `git status`,也让基于 tar 的同步因 `Cannot utime: Operation not permitted` 失败并在 `set -e` 下中止部署(实测踩到过) |
| 部署前校验本地已推送 | `scripts/deploy_to_server.sh` | 服务器只从 `origin/main` 取代码,未推送的提交不会被部署。脚本会检查 `HEAD` 是否为 `origin/main` 的祖先,否则拒绝执行(可用 `--allow-unpushed` 跳过),避免"改了代码却没生效"的困惑 |

### 建议但本次未做的修复

| 建议 | 位置 | 收益 |
|---|---|---|
| 放宽协调器状态查询超时 | `lifecycle_coordinator_node.cpp`:`fut.wait_for(100ms)` → 500ms;去掉 `service_is_ready()` 提前 return 或改为重试 | 让 `/system/ready` 真实反映状态,自动提拉生效。见第 9 节 |
| 让 `safety_gate_node` 周期发布 `/safety/state` | `safety_gate_node.cpp`:在 `WatchdogTick()`(100ms 定时器)里也调用 `PublishSafetyState` | 空闲期也有安全状态可观测,下游健康判定不再落空 |
| 启动时对受限后端做 fall-through | `backend_degradation.cpp`:`SwitchTo()` 在 `Open()` 失败时继续尝试下一个后端 | 无串口时自动落到 mock,不再依赖外部 PTY 模拟器 |
