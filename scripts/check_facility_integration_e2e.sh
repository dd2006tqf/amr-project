#!/usr/bin/env bash
# ==============================================================================
# check_facility_integration_e2e.sh - Phase 8 工业物理设施联锁集成验收测试
#
# 验收目标 (Phase 8 P3):
#   Stage 1: 启动 Mock PLC Modbus-TCP 服务端 (端口 15025)
#   Stage 2: 验证 Modbus-TCP Client 通信报文、读写线圈与保持寄存器
#   Stage 3: 验证 Facility Lease 超时自动回收 (防止僵尸锁)
#   Stage 4: 验证 Facility Interlocking Planning 状态机与超时降级机制:
#            - 靠近设施时: 申请设施锁并发送开门指令 (kAcquireAndOpen)
#            - 门开启中: 车辆就地停车等待 (kWaitInPlace)
#            - 门完全打开: 放行通行 (kProceed)
#            - 车辆通过后: 关门并释放锁 (kProceed + should_close=true, should_release_lock=true)
#            - 门卡死故障: 触发安全降级报警 (kTimeoutDegrade)
#   Stage 5: 调度器 ROS 服务端联调 (/v2/reserve_facility_resource) 契约验证
#
# 环境隔离:
#   PLC Port: 15025
#   ROS Domain: 85
# ==============================================================================

set -eo pipefail
set -E
trap 'echo "[FACILITY-E2E-FATAL] line=${LINENO} status=$? cmd=${BASH_COMMAND}" >&2' ERR

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# 内层守卫：宿主机无 ros2 且未标记内层执行 -> 穿透进容器
# CI 的 e2e-fleet job 直接在 ros:jazzy-ros-base 容器内以 AMR_E2E_INNER=1 运行，
# 此时不会（也不能）再调 docker exec（容器内无 docker socket）。
if [ -z "${AMR_E2E_INNER:-}" ] && ! command -v ros2 >/dev/null 2>&1; then
    echo "[E2E-WRAPPER] 检测到宿主机环境，正在穿透入 Docker amr_dispatcher 容器执行..."
    exec docker exec -i -e AMR_E2E_INNER=1 -w /workspace amr_dispatcher \
        bash /workspace/scripts/check_facility_integration_e2e.sh "$@"
fi

# core 构建目录解析（宿主机 build/core vs colcon build/amr_dispatcher_core 的
# CTest 文件内嵌了各自构建时的绝对路径，跨挂载点不能混用）。
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

# 直接执行 gtest 二进制（不经 ctest：ctest 会读构建期写死的绝对路径）
run_core_test() {
    local name="$1"
    if [ ! -x "${CORE_BUILD}/${name}" ]; then
        echo "[FAIL] 缺少测试可执行文件: ${CORE_BUILD}/${name}" >&2
        exit 1
    fi
    "${CORE_BUILD}/${name}"
}

PLC_PORT=15025
TMP_DIR="$(mktemp -d -t amr_facility_e2e_XXXXXX)"
PLC_PID=""

cleanup() {
    if [ -n "${PLC_PID}" ] && kill -0 "${PLC_PID}" 2>/dev/null; then
        echo ">>> 停止 Mock PLC 服务端 (PID: ${PLC_PID})..."
        kill -9 "${PLC_PID}" 2>/dev/null || true
    fi
    rm -rf "${TMP_DIR}"
}
trap cleanup EXIT

echo "======================================================================"
echo " Phase 8: Industrial Facility Integration & Protocol Adapters E2E Test"
echo " Workspace: ${REPO_ROOT}"
echo " PLC Port:  ${PLC_PORT}"
echo " TMP Dir:   ${TMP_DIR}"
echo "======================================================================"

echo ">>> [Stage 1/5] 启动现场 PLC 模拟器 (Modbus-TCP)..."
python3 "${REPO_ROOT}/scripts/mock_plc_facility.py" --port "${PLC_PORT}" --door-delay 0.5 > "${TMP_DIR}/plc.log" 2>&1 &
PLC_PID=$!
sleep 1

if ! kill -0 "${PLC_PID}" 2>/dev/null; then
    echo "[FAIL] Mock PLC 启动失败，日志:"
    cat "${TMP_DIR}/plc.log"
    exit 1
fi
echo "[OK] Mock PLC 正在运行 (PID: ${PLC_PID})"

echo ">>> [Stage 2/5] 执行纯 C++ Modbus-TCP 报文与适配器契约测试..."
run_core_test test_modbus_tcp_client
echo "[OK] Modbus-TCP 报文编解码与异常处理测试全部通过。"

echo ">>> [Stage 3/5] 验证设施租约超时自动回收 (Facility Lease Guard)..."
run_core_test test_facility_lease
echo "[OK] 设施超时租约自动回收与两阶段置换断言通过。"

echo ">>> [Stage 4/5] 验证多车在途设施联锁工作流与超时降级逻辑..."
run_core_test test_facility_interlocking_planning
echo "[OK] 设施开闭门闭环、跨楼层电梯状态轮询及卡死降级断言通过。"

echo ">>> [Stage 5/5] 执行全包测试回归..."
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

echo "======================================================================"
echo " Phase 8 Industrial Facility Integration E2E: ALL 5 STAGES PASSED"
echo "======================================================================"
