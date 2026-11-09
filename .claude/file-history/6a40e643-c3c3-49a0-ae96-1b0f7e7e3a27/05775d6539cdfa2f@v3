#!/usr/bin/env bash
# check_traffic_coordination_e2e.sh
# Phase 7 (P3): 运行时交通协调与在途死锁恢复集成验证脚本
# 隔离运行域: ROS_DOMAIN_ID=89, REST 18097, MQTT 18899
#
# CI 可移植性说明
# ---------------
# 本脚本必须同时满足两种执行形态：
#   1. 宿主机（已起 amr_dispatcher 容器）：穿透进容器执行（本文件的内层守卫）；
#   2. CI 的 e2e-fleet job：脚本本身就跑在 ros:jazzy-ros-base 容器内，
#      以 AMR_E2E_INNER=1 直接执行，此时**不能**再调 docker exec（容器内无 docker socket）。
# 因此所有构建目录一律用 ${REPO_ROOT}/build/core 相对定位，绝不硬编码宿主机绝对路径。
set -eo pipefail
set -E
trap 'echo "[TRAFFIC-E2E-FATAL] line=${LINENO} status=$? cmd=${BASH_COMMAND}" >&2' ERR

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# 内层守卫：宿主机无 ros2 且未标记内层执行 -> 穿透进容器
if [ -z "${AMR_E2E_INNER:-}" ] && ! command -v ros2 >/dev/null 2>&1; then
    echo "[E2E-WRAPPER] 检测到宿主机环境，正在穿透入 Docker amr_dispatcher 容器执行..."
    exec docker exec -i -e AMR_E2E_INNER=1 -w /workspace amr_dispatcher \
        bash /workspace/scripts/check_traffic_coordination_e2e.sh "$@"
fi

DOMAIN_ID="${AMR_E2E_DOMAIN_ID:-89}"
REST_PORT="${AMR_E2E_REST_PORT:-18097}"
MQTT_PORT="${AMR_E2E_MQTT_PORT:-18899}"
# core 构建目录解析（两种构建形态的绝对路径不同，不能混用）：
#   宿主机 scripts/build.sh -> ${REPO_ROOT}/build/core          （CTest 文件内嵌宿主机绝对路径）
#   colcon（容器 / CI）     -> ${REPO_ROOT}/build/amr_dispatcher_core（内嵌 /workspace 绝对路径）
# 判据用 /workspace 是否存在：它在容器里一定存在，在宿主机上不存在。
if [ -d /workspace ]; then
    CORE_BUILD="${REPO_ROOT}/build/amr_dispatcher_core"
    [ -d "${CORE_BUILD}" ] || CORE_BUILD="${REPO_ROOT}/build/core"
else
    CORE_BUILD="${REPO_ROOT}/build/core"
    [ -d "${CORE_BUILD}" ] || CORE_BUILD="${REPO_ROOT}/build/amr_dispatcher_core"
fi
if [ ! -d "${CORE_BUILD}" ]; then
    echo "[FAIL] 未找到 core 构建目录（build/core 或 build/amr_dispatcher_core）" >&2
    exit 1
fi

echo "=== Phase 7: Runtime Traffic Coordination & Deadlock Recovery E2E Test ==="
echo "Domain: ${DOMAIN_ID}, REST: ${REST_PORT}, MQTT: ${MQTT_PORT}"
echo "Workspace: ${REPO_ROOT}"
echo "Core build: ${CORE_BUILD}"

# 1. 直接执行测试二进制（不经 ctest：ctest 会读取构建期写死的绝对路径，
#    跨宿主机/容器挂载点会解析失败；gtest 二进制本身与路径无关）。
echo "[1/3] Running core traffic coordination unit tests..."
for bin in test_traffic_coordination_model test_in_flight_reroute_planning \
           test_traffic_recovery_coordinator; do
    if [ ! -x "${CORE_BUILD}/${bin}" ]; then
        echo "[FAIL] 缺少测试可执行文件: ${CORE_BUILD}/${bin}" >&2
        exit 1
    fi
    "${CORE_BUILD}/${bin}"
done

# 2. ROS 包契约测试：内层执行时直接 colcon test，否则穿透容器
echo "[2/3] Verifying ROS package contracts..."
if [ -n "${AMR_E2E_INNER:-}" ] || command -v colcon >/dev/null 2>&1; then
    ( cd "${REPO_ROOT}" \
      && set +u && source /opt/ros/jazzy/setup.bash && set -u \
      && colcon test --base-paths src --packages-select amr_dispatcher_core amr_dispatcher_ros \
      && colcon test-result --all )
else
    docker exec amr_dispatcher /bin/bash -c \
        "source /opt/ros/jazzy/setup.bash && colcon test --base-paths src \
         --packages-select amr_dispatcher_core amr_dispatcher_ros && colcon test-result --all"
fi

# 3. 断言两阶段资源置换与死锁恢复的不变量
# (a) 成功绕行时：新锁必须被预约，旧未来锁必须被释放，当前安全节点锁必须严格保留
# (b) 预约冲突时：新路线不下发，旧锁严格保留，任务进入等待或重试状态
# 第 1 步已逐个执行上面三个二进制（含全部断言），此处不再重复执行。
echo "[3/3] Two-phase swap & recovery invariants verified above (step 1)."

echo "=== Phase 7 Traffic Coordination E2E: ALL CHECKS PASSED ==="
