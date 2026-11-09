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
if [ -f "${REPO_ROOT}/build/core/test_modbus_tcp_client" ]; then
    "${REPO_ROOT}/build/core/test_modbus_tcp_client"
else
    ctest --test-dir "${REPO_ROOT}/build/core" -R "test_modbus_tcp_client" --output-on-failure
fi
echo "[OK] Modbus-TCP 报文编解码与异常处理测试全部通过。"

echo ">>> [Stage 3/5] 验证设施租约超时自动回收 (Facility Lease Guard)..."
if [ -f "${REPO_ROOT}/build/core/test_facility_lease" ]; then
    "${REPO_ROOT}/build/core/test_facility_lease"
else
    ctest --test-dir "${REPO_ROOT}/build/core" -R "test_facility_lease" --output-on-failure
fi
echo "[OK] 设施超时租约自动回收与两阶段置换断言通过。"

echo ">>> [Stage 4/5] 验证多车在途设施联锁工作流与超时降级逻辑..."
if [ -f "${REPO_ROOT}/build/core/test_facility_interlocking_planning" ]; then
    "${REPO_ROOT}/build/core/test_facility_interlocking_planning"
else
    ctest --test-dir "${REPO_ROOT}/build/core" -R "test_facility_interlocking_planning" --output-on-failure
fi
echo "[OK] 设施开闭门闭环、跨楼层电梯状态轮询及卡死降级断言通过。"

echo ">>> [Stage 5/5] 执行全包容器内回归检验..."
docker exec amr_dispatcher /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon test --base-paths src --packages-select amr_dispatcher_core amr_dispatcher_ros && colcon test-result --all"

echo "======================================================================"
echo " Phase 8 Industrial Facility Integration E2E: ALL 5 STAGES PASSED"
echo "======================================================================"
