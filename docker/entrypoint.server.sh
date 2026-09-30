#!/usr/bin/env bash
# AMR Dispatcher 容器入口脚本
#
# 职责
# ----
# 1. 源 ROS 2 Jazzy 环境与工作区 install/。
# 2. 按需 colcon 构建（接口包必须先构建，见下方说明）。
# 3. 无物理串口设备时，启动项目自带的 PTY 物理底盘模拟器，并把解析出的
#    /dev/pts/N 作为 serial_device 传给 launch。
# 4. 从工作区根目录启动 full_system.launch.py。
#
# 为什么必须 cd 到工作区根目录
# ---------------------------
# rest_gateway 与 dispatcher_visualizer_node 用相对路径加载静态资源：
#   rest_gateway.cpp:288            -> tools/operator_console.html
#   dispatcher_visualizer_node.cpp  -> config/stations.yaml
# 若工作目录不对，Web 控制台会 404，可视化节点会回退到 4 个硬编码默认站点。
#
# 用法
# ----
# 默认（自动构建 + 起模拟器 + 启动完整系统）:
#   docker compose -f docker/docker-compose.server.yml up -d
#
# 传参则直接执行该命令（跳过启动逻辑）:
#   docker run --rm -it -v "$PWD":/workspace amr-dispatcher:jazzy bash
#   docker run --rm    -v "$PWD":/workspace amr-dispatcher:jazzy bash -lc ./scripts/check_all.sh
#
# 环境变量
# --------
#   AMR_BUILD=1              强制重新构建
#   AMR_USE_MOCK_CHASSIS=1   无串口时启动 PTY 模拟器（默认 1）
#   AMR_SERIAL_DEVICE        指定串口设备（默认 /dev/ttyUSB0）
#   AMR_HEALTH_RATE_HZ       底盘健康/心跳发布频率（默认 5.0）
#   AMR_COLCON_WORKERS       colcon 并发数（默认 nproc；低内存机器建议设 1 或 2）

set -eo pipefail

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"

WORKSPACE="${AMR_WORKSPACE:-/workspace}"
cd "$WORKSPACE"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"

log() { echo "[amr-entrypoint] $*"; }

# ---------------------------------------------------------------- 构建
need_build=0
if [ "${AMR_BUILD:-0}" = "1" ]; then
  need_build=1
elif [ ! -f install/setup.bash ]; then
  log "未发现 install/setup.bash，执行首次构建"
  need_build=1
fi

if [ "$need_build" = "1" ]; then
  workers="${AMR_COLCON_WORKERS:-$(nproc)}"
  log "开始 colcon 构建（并行度 $workers）"

  # --base-paths src 是必需的：仓库根目录有一个 CMakeLists.txt，若不加此参数
  # colcon 会把【整个仓库根】当成一个包（amr_dispatcher_workspace），转而忽略
  # src/ 下的 5 个真实包，导致 install/ 里没有任何 amr_dispatcher_* 包，
  # 后续 `ros2 launch` 直接报 "package 'amr_dispatcher_ros' not found"。
  # 该组合与 scripts/container_build.sh 中的用法一致。
  BASE_ARGS=(--base-paths src --symlink-install)

  # 两阶段：amr_dispatcher_bt / _ros / _tools 依赖 amr_dispatcher_interfaces 生成的
  # 消息头文件，必须先单独构建 interfaces（连同 core）。
  colcon build \
    "${BASE_ARGS[@]}" \
    --packages-select amr_dispatcher_core amr_dispatcher_interfaces \
    --parallel-workers "$workers" \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

  # shellcheck disable=SC1091
  source install/setup.bash

  colcon build \
    "${BASE_ARGS[@]}" \
    --packages-select amr_dispatcher_bt amr_dispatcher_ros amr_dispatcher_tools \
    --parallel-workers "$workers" \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

  # 注意：此处不能用 `ros2 pkg list | grep -c`，因为 `source install/setup.bash`
  # 的输出会混入 grep，计数会偏低（实测 5 个包被数成 2 个）。
  # 直接看 install/ 目录更可靠。
  installed=$(find install -maxdepth 1 -type d -name 'amr_dispatcher_*' 2>/dev/null | wc -l)
  log "构建完成，install/ 下已安装 $installed 个项目包"
fi

if [ -f install/setup.bash ]; then
  # shellcheck disable=SC1091
  source install/setup.bash
fi

# ------------------------------------------------- 传入命令则直接执行
if [ "$#" -gt 0 ]; then
  exec "$@"
fi

# ------------------------------------------------------- PTY 底盘模拟器
SERIAL_DEVICE="${AMR_SERIAL_DEVICE:-/dev/ttyUSB0}"
HEALTH_RATE_HZ="${AMR_HEALTH_RATE_HZ:-5.0}"
MOCK_PID=""

cleanup() {
  if [ -n "$MOCK_PID" ] && kill -0 "$MOCK_PID" 2>/dev/null; then
    log "停止 PTY 底盘模拟器 (pid=$MOCK_PID)"
    kill "$MOCK_PID" 2>/dev/null || true
    wait "$MOCK_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

if [ "${AMR_USE_MOCK_CHASSIS:-1}" = "1" ] && [ ! -e "$SERIAL_DEVICE" ]; then
  log "$SERIAL_DEVICE 不存在，启动 PTY 物理底盘模拟器"

  if [ ! -f scripts/mock_physical_chassis.py ]; then
    log "WARN 找不到 scripts/mock_physical_chassis.py，跳过模拟器"
  else
    MOCK_LOG="$(mktemp)"
    # -u 关闭 Python 输出缓冲。不加这个参数时，stdout 重定向到文件会变为块缓冲
    # （默认 8KB），而脚本启动日志只有几百字节，导致解析端在轮询窗口内读不到任何
    # 内容，误判为"未能解析 PTY"，底盘拿不到串口设备。
    python3 -u scripts/mock_physical_chassis.py >"$MOCK_LOG" 2>&1 &
    MOCK_PID=$!

    # 轮询模拟器输出，解析它创建的 PTY 从设备路径
    PTY=""
    for _ in $(seq 1 120); do
      PTY="$(grep -oE '/dev/pts/[0-9]+' "$MOCK_LOG" 2>/dev/null | head -1 || true)"
      if [ -n "$PTY" ] && [ -e "$PTY" ]; then
        break
      fi
      PTY=""
      if ! kill -0 "$MOCK_PID" 2>/dev/null; then
        log "WARN 模拟器进程已退出，输出如下："
        cat "$MOCK_LOG" || true
        break
      fi
      sleep 0.25
    done

    if [ -n "$PTY" ]; then
      SERIAL_DEVICE="$PTY"
      log "模拟器就绪，serial_device=$SERIAL_DEVICE"
    else
      log "WARN 未能解析模拟器 PTY，沿用 $SERIAL_DEVICE（底盘将无数据，但心跳仍正常）"
    fi
  fi
else
  log "使用物理串口设备 $SERIAL_DEVICE"
fi

# -------------------------------------------------------------- 启动
log "启动完整系统（工作目录 $PWD，ROS_DOMAIN_ID=$ROS_DOMAIN_ID）"
ros2 launch launch/full_system.launch.py \
  serial_device:="$SERIAL_DEVICE" \
  health_rate_hz:="$HEALTH_RATE_HZ"
