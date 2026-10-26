#!/usr/bin/env bash
# ==============================================================================
# check_web_console_e2e.sh - Web 操作台（tools/operator_console.html）浏览器端验收
#
# 目标：把计划 C4 的"控制台手工验证"变成**可复现的自动化验收**。
#   浏览器端不再依赖人工肉眼点按钮——用 CDP 驱动 headless Chrome 真实点击，
#   断言受管车辆面板渲染 + 强清/恢复的**服务端复验**与**前端前置校验**。
#
# 关键断言（对应评审问题 6 的"前端隐藏按钮不是安全边界"）：
#   * 在线车强清 -> **服务端**拒绝（409），页面显示被拒原因
#   * 离线车强清 -> 受理，车辆转入**隔离态**，面板显示隔离
#   * 恢复未勾选"已核实物理位置" -> **前端**拦下（不发请求）
#   * 恢复（已勾选）-> 解除隔离
#
# 拓扑：ROS 栈（mosquitto / dispatcher / gateway / rest_gateway / mock VDA 车）
#       跑在 Docker 容器 amr_dispatcher 内（域 82 / MQTT 18890 / REST 18092）；
#       headless Chrome + CDP 跑在**宿主机**（容器内无 node/chrome），
#       通过容器 IP 直连 REST 网关。
#
# 环境隔离：域 82 / MQTT 18890 / REST 18092，与生产环境（域 42 / 8080）
#           及 VDA 5050 e2e（域 77/78/80）完全隔离，不产生串扰。
#
# 退出码：0 = 全部断言通过；非 0 = 有断言失败（现场保留在 /tmp 便于排查）
# ==============================================================================

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

CONTAINER="${AMR_WC_CONTAINER:-amr_dispatcher}"
WC_DOMAIN_ID="${AMR_WC_DOMAIN_ID:-82}"
WC_MQTT_PORT="${AMR_WC_MQTT_PORT:-18890}"
WC_REST_PORT="${AMR_WC_REST_PORT:-18092}"
WC_CDP_PORT="${AMR_WC_CDP_PORT:-9333}"
WC_VEHICLE_ID="${AMR_WC_VEHICLE_ID:-wc_amr_01}"
CONTAINER_WORKDIR="${AMR_WC_WORKDIR:-/workspace}"

for tool in node curl docker; do
    command -v "${tool}" >/dev/null 2>&1 || {
        echo "[WC-FAIL] 宿主机缺少必需工具: ${tool}" >&2
        exit 2
    }
done
CHROME_BIN="$(command -v google-chrome || command -v chromium || command -v chromium-browser || true)"
[ -n "${CHROME_BIN}" ] || {
    echo "[WC-FAIL] 宿主机未找到 Chrome/Chromium（headless 验收需要）" >&2
    exit 2
}
docker ps --format '{{.Names}}' | grep -qx "${CONTAINER}" || {
    echo "[WC-FAIL] 容器 ${CONTAINER} 未运行" >&2
    exit 2
}

TMP_DIR="$(mktemp -d -t amr_web_console_XXXXXX)"
WC_FAILED=0

cleanup() {
    local exit_code=$?
    echo ""
    echo "[WC] 正在清理..."
    # 1) 停掉容器内本轮拉起的 ROS 栈（按域 ID 精确清扫，不碰生产域 42）
    docker exec "${CONTAINER}" bash -c "
        for envf in /proc/[0-9]*/environ; do
            pid=\"\${envf#/proc/}\"; pid=\"\${pid%/environ}\"
            [ \"\${pid}\" = \"\$\$\" ] && continue
            [ -r \"\${envf}\" ] || continue
            if tr '\\0' '\\n' 2>/dev/null < \"\${envf}\" | grep -qx 'ROS_DOMAIN_ID=${WC_DOMAIN_ID}'; then
                kill -TERM \"\${pid}\" 2>/dev/null || true
            fi
        done
        sleep 0.5
        for envf in /proc/[0-9]*/environ; do
            pid=\"\${envf#/proc/}\"; pid=\"\${pid%/environ}\"
            [ \"\${pid}\" = \"\$\$\" ] && continue
            [ -r \"\${envf}\" ] || continue
            if tr '\\0' '\\n' 2>/dev/null < \"\${envf}\" | grep -qx 'ROS_DOMAIN_ID=${WC_DOMAIN_ID}'; then
                kill -KILL \"\${pid}\" 2>/dev/null || true
            fi
        done
        ROS_DOMAIN_ID=${WC_DOMAIN_ID} bash -c 'source /opt/ros/jazzy/setup.bash >/dev/null 2>&1; ros2 daemon stop >/dev/null 2>&1' || true
        # mock VDA 车**不订阅 ROS 话题**（只走 MQTT），其进程环境里没有
        # ROS_DOMAIN_ID，上面的按域清扫抓不到它 —— 必须按 MQTT 端口精确清，
        # 否则上一轮的孤儿车会重连到本轮 broker，把"离线"车复活成在线。
        # 用 [m]osquitto / [m]ock 这种括号技巧：pkill -f 的模式串会出现在
        # 本 shell 自己的命令行里，直写会把调用方自己一起杀掉（先杀自己
        # 就来不及杀目标了）。生产 broker（端口 1883）**绝不在此模式内**。
        pkill -9 -f '[m]ock_vda5050_vehicle.py --host 127.0.0.1 --port ${WC_MQTT_PORT}' 2>/dev/null || true
        pkill -9 -f '[m]osquitto -c ${CONTAINER_WORKDIR}/.wc_stamp/mosquitto.conf' 2>/dev/null || true
    " >/dev/null 2>&1 || true
    # 2) 停掉宿主机 headless Chrome
    pkill -f "remote-debugging-port=${WC_CDP_PORT}" 2>/dev/null || true
    sleep 1  # 等 Chrome 落盘 profile 退出，再删目录
    if [ ${exit_code} -eq 0 ] && [ ${WC_FAILED} -eq 0 ]; then
        echo "[WC] 全部通过，清理临时目录: ${TMP_DIR}"
        rm -rf "${TMP_DIR}" 2>/dev/null || true
    else
        echo "[WC][WARN] 验收未全部成功，保留现场: ${TMP_DIR}"
    fi
    exit ${exit_code}
}
trap cleanup EXIT INT TERM

fail() {
    echo ""
    echo "[WC-FAIL] 验收失败: $1"
    echo "--- [LOG TAIL: dispatcher] ---"
    docker exec "${CONTAINER}" bash -c "tail -n 40 '${CONTAINER_WORKDIR}/.wc_stamp/dispatcher.log' 2>/dev/null" || echo "(no log)"
    echo "--- [LOG TAIL: gateway] ---"
    docker exec "${CONTAINER}" bash -c "tail -n 20 '${CONTAINER_WORKDIR}/.wc_stamp/gateway.log' 2>/dev/null" || echo "(no log)"
    echo "--- [LOG TAIL: chrome] ---"
    tail -n 20 "${TMP_DIR}/chrome.log" 2>/dev/null || echo "(no log)"
    WC_FAILED=1
    exit 1
}

CONTAINER_IP="$(docker inspect -f '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' "${CONTAINER}")"
[ -n "${CONTAINER_IP}" ] || fail "无法解析容器 IP"

echo "======================================================================"
echo " Web Operator Console End-to-End Acceptance (headless Chrome + CDP)"
echo " Container:     ${CONTAINER} (${CONTAINER_IP})"
echo " ROS_DOMAIN_ID: ${WC_DOMAIN_ID}  MQTT: ${WC_MQTT_PORT}  REST: ${WC_REST_PORT}"
echo " TMP_DIR:       ${TMP_DIR}"
echo "======================================================================"

STATIONS_YAML="${REPO_ROOT}/config/generated/stations.yaml"
[ -f "${STATIONS_YAML}" ] || fail "缺少 ${STATIONS_YAML}（先跑 check_map_ingest_e2e.sh 生成拓扑）"
MAP_HASH="$(grep -oE 'fnv1a64_[0-9a-f]+' "${STATIONS_YAML}" | head -1)"
[ -n "${MAP_HASH}" ] || fail "无法从 stations.yaml 提取 map_hash"

# 容器内的 stations.yaml 路径（宿主机仓库根 != 容器内 /workspace）
CONTAINER_STATIONS_YAML="${CONTAINER_WORKDIR}/config/generated/stations.yaml"
docker exec "${CONTAINER}" test -f "${CONTAINER_STATIONS_YAML}" \
    || fail "容器内缺少 ${CONTAINER_STATIONS_YAML}"

# ---------------------------------------------------------------------------
# 在容器内拉起本轮的 ROS 栈（独立域，与生产完全隔离）
# ---------------------------------------------------------------------------
docker exec "${CONTAINER}" bash -c "rm -rf '${CONTAINER_WORKDIR}/.wc_stamp' && mkdir -p '${CONTAINER_WORKDIR}/.wc_stamp'"

# 内层栈脚本：起 broker / dispatcher / gateway / rest，等待就绪后落 READY 标记
# 写成文件再执行：docker exec -d 后台跑，避免外层被 inner 的 sleep 阻塞。
cat > "${TMP_DIR}/stack_inner.sh" <<INNER
set -eo pipefail
cd "${CONTAINER_WORKDIR}"
source /opt/ros/jazzy/setup.bash
source "${CONTAINER_WORKDIR}/install/setup.bash"
export ROS_DOMAIN_ID=${WC_DOMAIN_ID}
export PYTHONUNBUFFERED=1
STAMP="${CONTAINER_WORKDIR}/.wc_stamp"

cat > "\${STAMP}/mosquitto.conf" <<EOF
listener ${WC_MQTT_PORT} 127.0.0.1
allow_anonymous true
EOF
mosquitto -c "\${STAMP}/mosquitto.conf" -v > "\${STAMP}/mosq.log" 2>&1 &
sleep 1

ros2 run amr_dispatcher_ros dispatcher_node --ros-args \\
    -p stations_file:="${CONTAINER_STATIONS_YAML}" \\
    -p planner_type:=skeleton \\
    -p expected_map_hash:="${MAP_HASH}" \\
    -p vda5050_vehicles:="${WC_VEHICLE_ID}" \\
    -p vda5050_state_stale_sec:=5.0 \\
    -p traffic_mode:=time_space \\
    > "\${STAMP}/dispatcher.log" 2>&1 &

for ((i = 0; i < 120; i++)); do
    if ros2 node list 2>/dev/null | grep -q "/dispatcher_node"; then break; fi
    sleep 0.1
done
ros2 lifecycle set /dispatcher_node configure >/dev/null 2>&1
ros2 lifecycle set /dispatcher_node activate >/dev/null 2>&1

ros2 run amr_dispatcher_ros vda5050_gateway_node --ros-args \\
    -p use_vda5050:=true \\
    -p mqtt_broker_host:=127.0.0.1 \\
    -p mqtt_port:="${WC_MQTT_PORT}" \\
    -p manufacturer:=RobotManufacturer \\
    -p serial_number:=wc_gateway \\
    -p protocol_version:=v2 \\
    -p heartbeat_ms:=5000 \\
    > "\${STAMP}/gateway.log" 2>&1 &

ros2 run amr_dispatcher_tools rest_gateway --ros-args \\
    -p port:="${WC_REST_PORT}" \\
    -p html_path:="${CONTAINER_WORKDIR}/tools/operator_console.html" \\
    > "\${STAMP}/rest.log" 2>&1 &

# 等待 REST 端口就绪
for ((i = 0; i < 150; i++)); do
    if bash -c "cat < /dev/null > /dev/tcp/127.0.0.1/${WC_REST_PORT}" 2>/dev/null; then
        echo READY > "\${STAMP}/READY"
        break
    fi
    sleep 0.2
done
sleep 3600
INNER

docker cp "${TMP_DIR}/stack_inner.sh" "${CONTAINER}:${CONTAINER_WORKDIR}/.wc_stamp/stack_inner.sh" >/dev/null
docker exec -d "${CONTAINER}" bash "${CONTAINER_WORKDIR}/.wc_stamp/stack_inner.sh" \
    > "${TMP_DIR}/stack_launch.log" 2>&1
echo "[WC] 容器内 ROS 栈已拉起（域 ${WC_DOMAIN_ID}）"

# 等内层栈就绪
READY=0
for ((i = 0; i < 200; i++)); do
    if docker exec "${CONTAINER}" bash -c "test -f '${CONTAINER_WORKDIR}/.wc_stamp/READY'" 2>/dev/null; then
        READY=1
        break
    fi
    sleep 0.3
done
[ "${READY}" -eq 1 ] || fail "容器内 rest_gateway 未在预期时间内就绪（域 ${WC_DOMAIN_ID}）"
echo "[WC] rest_gateway 就绪于 ${CONTAINER_IP}:${WC_REST_PORT} (PASS)"

CONSOLE_BASE="http://${CONTAINER_IP}:${WC_REST_PORT}"

# ---------------------------------------------------------------------------
# 宿主机：headless Chrome + CDP
# ---------------------------------------------------------------------------
rm -rf "${TMP_DIR}/chrome-profile"
"${CHROME_BIN}" --headless=new --disable-gpu --no-sandbox \
    --disable-dev-shm-usage \
    --remote-debugging-port="${WC_CDP_PORT}" \
    --user-data-dir="${TMP_DIR}/chrome-profile" \
    about:blank > "${TMP_DIR}/chrome.log" 2>&1 &

CDP_READY=0
for ((i = 0; i < 100; i++)); do
    if curl -s "http://127.0.0.1:${WC_CDP_PORT}/json/version" >/dev/null 2>&1; then
        CDP_READY=1
        break
    fi
    sleep 0.2
done
[ "${CDP_READY}" -eq 1 ] || fail "headless Chrome 的 CDP 端口 ${WC_CDP_PORT} 未就绪"
echo "[WC] headless Chrome CDP 就绪于 ${WC_CDP_PORT} (PASS)"

run_phase() {
    local phase="$1"
    echo ""
    echo "[WC] 浏览器端断言: ${phase}"
    local out
    if ! out="$(CDP_PORT="${WC_CDP_PORT}" node "${SCRIPT_DIR}/web_console_check.mjs" "${phase}" "${CONSOLE_BASE}")"; then
        echo "${out}"
        fail "浏览器端断言失败: ${phase}"
    fi
    echo "${out}"
}

# ---------------------------------------------------------------------------
# 阶段 1：车辆上线 -> 面板渲染
# ---------------------------------------------------------------------------
docker exec -d "${CONTAINER}" bash -c "
    source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
    cd ${CONTAINER_WORKDIR}
    exec python3 scripts/mock_vda5050_vehicle.py --host 127.0.0.1 --port ${WC_MQTT_PORT} \
        --manufacturer RobotManufacturer --serial ${WC_VEHICLE_ID} --speed 0.6 --battery 87.5 \
        > ${CONTAINER_WORKDIR}/.wc_stamp/vehicle.log 2>&1
"

LINKS_OK=0
for ((i = 0; i < 150; i++)); do
    if curl -s --max-time 3 "${CONSOLE_BASE}/api/v1/vehicle_links" 2>/dev/null | grep -q "${WC_VEHICLE_ID}"; then
        LINKS_OK=1
        break
    fi
    sleep 0.2
done
[ "${LINKS_OK}" -eq 1 ] || fail "REST /api/v1/vehicle_links 未返回受管车 ${WC_VEHICLE_ID}"
echo "[WC] 受管车已上线并出现在 REST 视图 (PASS)"

run_phase panel

# ---------------------------------------------------------------------------
# 阶段 2：在线车强清 -> 服务端必须拒绝
# ---------------------------------------------------------------------------
run_phase force_clear_online

# ---------------------------------------------------------------------------
# 阶段 3：物理下线（kill -9 -> LWT CONNECTIONBROKEN）-> 强清 -> 隔离
# ---------------------------------------------------------------------------
VEHICLE_PID="$(docker exec "${CONTAINER}" bash -c "pgrep -f 'mock_vda5050_vehicle.py --host 127.0.0.1 --port ${WC_MQTT_PORT}' | head -1" | tr -d '\r')"
[ -n "${VEHICLE_PID}" ] || fail "未找到 mock VDA 车进程"
docker exec "${CONTAINER}" bash -c "kill -9 ${VEHICLE_PID}" >/dev/null 2>&1 || true

OFFLINE_OK=0
for ((i = 0; i < 150; i++)); do
    if curl -s --max-time 3 "${CONSOLE_BASE}/api/v1/vehicle_links" 2>/dev/null | grep -q '"online":false'; then
        OFFLINE_OK=1
        break
    fi
    sleep 0.2
done
[ "${OFFLINE_OK}" -eq 1 ] || fail "REST 未反映车辆离线状态"
echo "[WC] 车辆已离线（kill -9 -> LWT CONNECTIONBROKEN） (PASS)"

run_phase force_clear_offline

# ---------------------------------------------------------------------------
# 阶段 4：恢复 —— 前端前置校验 + 显式恢复
# ---------------------------------------------------------------------------
run_phase restore_guard

# 让车重新上线，才能通过服务端的"车辆在线"复验
docker exec -d "${CONTAINER}" bash -c "
    source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
    cd ${CONTAINER_WORKDIR}
    exec python3 scripts/mock_vda5050_vehicle.py --host 127.0.0.1 --port ${WC_MQTT_PORT} \
        --manufacturer RobotManufacturer --serial ${WC_VEHICLE_ID} --speed 0.6 --battery 87.5 \
        > ${CONTAINER_WORKDIR}/.wc_stamp/vehicle2.log 2>&1
"

ONLINE_AGAIN=0
for ((i = 0; i < 150; i++)); do
    if curl -s --max-time 3 "${CONSOLE_BASE}/api/v1/vehicle_links" 2>/dev/null | grep -q '"online":true'; then
        ONLINE_AGAIN=1
        break
    fi
    sleep 0.2
done
[ "${ONLINE_AGAIN}" -eq 1 ] || fail "车辆未恢复在线"
echo "[WC] 车辆已重新上线 (PASS)"

run_phase restore

echo ""
echo "======================================================================"
echo " [PASS] Web 控制台浏览器端验收全部通过:"
echo "                 - 受管车辆面板渲染 (表格/后端/在线态/电量/状态时延)"
echo "                 - 在线车强清被**服务端**拒绝 (前端隐藏按钮不是安全边界)"
echo "                 - 缺审计字段被前端拦下 (vehicle_id/operator_id/reason)"
echo "                 - 离线车强清受理 -> 车辆转入隔离态 (面板可见)"
echo "                 - 恢复未勾选已核实位置被前端拦下"
echo "                 - 显式恢复 -> 解除隔离, 车辆重回调度池"
echo "======================================================================"

exit 0
