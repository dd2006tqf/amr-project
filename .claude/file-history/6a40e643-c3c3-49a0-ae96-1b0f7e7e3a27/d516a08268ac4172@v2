#!/usr/bin/env bash
# ==============================================================================
# check_metrics_e2e.sh - /metrics 端点验收（Prometheus 文本格式）
#
# 断言目标：抓取端拿到的是**合法** exposition，而不是一段看起来像指标的文本：
#   * Content-Type 是 text/plain; version=0.0.4
#   * 每个指标都有 # HELP / # TYPE，且 TYPE 只取 gauge / counter
#   * 每行样本都能被 `name{label="value"} number` 完整匹配（标签值已转义）
#   * 数值不是科学计数法、不是空、不是 NaN（本场景所有指标都是有限值）
#   * 无受管车时**不**产生空族（没有 amr_vehicle_online 的 HELP 行）
#   * 网关自身请求计数可自证：请求过 401 与 200 后，两个 code 都有样本
#   * metrics_token 独立生效：/metrics 用 metrics_token，业务端点仍用 auth_token
#
# 环境隔离：ROS_DOMAIN_ID=88 / REST 18098，不属于生产（42 / 8080），也不与
#           其它 e2e（77/78/80/82/84/86/87）重叠。只起 rest_gateway，
#           不依赖 dispatcher（无 DispatcherState 时指标应退化为 0 而不是崩溃）。
#
# 退出码：0 = 全部通过；非 0 = 断言失败（现场保留在容器 /tmp 便于排查）
# ==============================================================================
set -eo pipefail
set -E
trap 'echo "[METRICS-FATAL] line=${LINENO} status=$? cmd=${BASH_COMMAND}" >&2' ERR

CONTAINER="${AMR_METRICS_CONTAINER:-amr_dispatcher}"
METRICS_DOMAIN_ID="${AMR_METRICS_DOMAIN_ID:-88}"
METRICS_PORT="${AMR_METRICS_PORT:-18098}"
CONTAINER_WORKDIR="${AMR_METRICS_WORKDIR:-/workspace}"
AUTH_TOKEN="metrics-e2e-auth-token"
METRICS_TOKEN="metrics-e2e-scrape-token"
ASSERT_SCRIPT="$(mktemp -t amr_metrics_assert_XXXXXX.py)"

for tool in docker python3; do
    command -v "${tool}" >/dev/null 2>&1 || {
        echo "[METRICS-FAIL] 宿主机缺少必需工具: ${tool}" >&2
        exit 2
    }
done
docker ps --format '{{.Names}}' | grep -qx "${CONTAINER}" || {
    echo "[METRICS-FAIL] 容器 ${CONTAINER} 未运行" >&2
    exit 2
}

# 生产红线：域 42 / 端口 8080 绝不触碰
if [ "${METRICS_DOMAIN_ID}" = "42" ] || [ "${METRICS_PORT}" = "8080" ]; then
    echo "[METRICS-FAIL] 拒绝使用生产域/端口 (42 / 8080)" >&2
    exit 2
fi

cleanup() {
    rm -f "${ASSERT_SCRIPT}"
    docker exec "${CONTAINER}" bash -c \
        "pkill -f 'rest_gateway.*port:=${METRICS_PORT}' 2>/dev/null; true" || true
}
trap 'cleanup' EXIT

echo "[Metrics] 容器 ${CONTAINER} / 域 ${METRICS_DOMAIN_ID} / 端口 ${METRICS_PORT}"

# ---- 0. 清扫同域遗留 ----
docker exec "${CONTAINER}" bash -c "
    for envf in /proc/[0-9]*/environ; do
        pid=\${envf#/proc/}; pid=\${pid%/environ}
        [ -r \"\$envf\" ] || continue
        if tr '\\0' '\\n' 2>/dev/null < \"\$envf\" | grep -qx 'ROS_DOMAIN_ID=${METRICS_DOMAIN_ID}'; then
            kill -KILL \"\$pid\" 2>/dev/null || true
        fi
    done
    true
"
echo "[Metrics] 已清扫域 ${METRICS_DOMAIN_ID} 的遗留进程"

# ---- 1. 起网关（metrics_token 与 auth_token 刻意不同）----
docker exec -d "${CONTAINER}" bash -c "
    export ROS_DOMAIN_ID=${METRICS_DOMAIN_ID}
    source /opt/ros/jazzy/setup.bash
    source ${CONTAINER_WORKDIR}/install/setup.bash
    exec ros2 run amr_dispatcher_tools rest_gateway --ros-args \
        -p port:=${METRICS_PORT} \
        -p auth_token:=${AUTH_TOKEN} \
        -p metrics_token:=${METRICS_TOKEN}
"

for _ in $(seq 1 100); do
    if docker exec "${CONTAINER}" bash -c \
            "cat < /dev/null > /dev/tcp/127.0.0.1/${METRICS_PORT}" 2>/dev/null; then
        break
    fi
    sleep 0.1
done
docker exec "${CONTAINER}" bash -c \
    "cat < /dev/null > /dev/tcp/127.0.0.1/${METRICS_PORT}" 2>/dev/null \
    || { echo "[METRICS-FAIL] 网关未在 10s 内监听 ${METRICS_PORT}" >&2; exit 1; }
echo "[Metrics] 网关已监听 127.0.0.1:${METRICS_PORT}"

# ---- 2. 断言脚本落在宿主机文件里再送进容器执行 ----
# 不把 python 源码直接嵌进 `docker exec bash -c "..."`：外层双引号会改写
# 反斜杠，而本脚本的正则与转义断言恰恰依赖反斜杠原样传递。
cat > "${ASSERT_SCRIPT}" <<'PY'
import re
import sys
import urllib.error
import urllib.request

port, metrics_token = sys.argv[1:3]
base = f"http://127.0.0.1:{port}"
failures = []


def check(cond, message):
    if cond:
        print(f"  [OK] {message}")
    else:
        failures.append(message)
        print(f"  [FAIL] {message}")


def get(path, token=None):
    headers = {"Authorization": f"Bearer {token}"} if token else {}
    req = urllib.request.Request(base + path, headers=headers)
    try:
        with urllib.request.urlopen(req, timeout=10) as resp:
            return resp.status, resp.headers.get("Content-Type", ""), resp.read().decode()
    except urllib.error.HTTPError as exc:
        return exc.code, exc.headers.get("Content-Type", ""), exc.read().decode()
    except Exception as exc:  # 连接失败也要给出可读结论
        return 0, "", f"ERROR: {exc}"


# 2.1 未带口令 / 错口令 -> 401（metrics_token 已配置）
code, _, _ = get("/metrics")
check(code == 401, f"/metrics 无口令应 401，实际 {code}")
code, _, _ = get("/metrics", "wrong-token")
check(code == 401, f"/metrics 错口令应 401，实际 {code}")

# 2.2 口令隔离：业务端点不接受 metrics_token
code, _, _ = get("/api/v1/vehicle_links", metrics_token)
check(code == 401, f"/api/v1/vehicle_links 不接受 metrics_token，实际 {code}")

# 2.3 正确 metrics_token -> 200 + text/plain; version=0.0.4
code, ctype, body = get("/metrics", metrics_token)
check(code == 200, f"/metrics 正确口令应 200，实际 {code}")
check("text/plain" in ctype, f"Content-Type 应为 text/plain，实际 {ctype!r}")
check("version=0.0.4" in ctype, f"Content-Type 应带 version=0.0.4，实际 {ctype!r}")

lines = [line for line in body.split("\n") if line != ""]

# name{key="value",key2="value2"} value —— 标签值内部允许 \\x 与 \" 转义
LABEL_SET = r'\{[a-zA-Z_][a-zA-Z0-9_]*="(?:[^"\\]|\\.)*"' \
            r'(?:,[a-zA-Z_][a-zA-Z0-9_]*="(?:[^"\\]|\\.)*")*\}'
SAMPLE_RE = re.compile(r'^([a-zA-Z_:][a-zA-Z0-9_:]*)(' + LABEL_SET + r')? (\S+)$')
helped = set()
typed = {}
sample_names = set()
bad_lines = []
for line in lines:
    if line.startswith("# HELP "):
        parts = line.split(" ", 3)
        if len(parts) < 4:
            bad_lines.append(line)
        else:
            helped.add(parts[2])
        continue
    if line.startswith("# TYPE "):
        parts = line.split(" ")
        if len(parts) != 4 or parts[3] not in ("gauge", "counter"):
            bad_lines.append(line)
        else:
            typed[parts[2]] = parts[3]
        continue
    match = SAMPLE_RE.match(line)
    if not match:
        bad_lines.append(line)
        continue
    sample_names.add(match.group(1))
    value = match.group(3)
    if value in ("NaN", "+Inf", "-Inf"):
        bad_lines.append(line + "  # 本场景不应出现非有限值")
        continue
    try:
        float(value)
    except ValueError:
        bad_lines.append(line + "  # 数值不可解析")
    if "e" in value.lower():
        bad_lines.append(line + "  # 不希望出现科学计数法")

check(not bad_lines,
      "所有行都是合法 exposition 行" if not bad_lines
      else "非法行: " + " | ".join(bad_lines[:5]))
check(bool(sample_names), "至少产生一个样本")
check(sample_names <= helped, f"每个样本都有 HELP（缺: {sorted(sample_names - helped)}）")
check(sample_names <= set(typed), f"每个样本都有 TYPE（缺: {sorted(sample_names - set(typed))}）")

# 2.4 核心调度指标齐全
for name in ("amr_queue_size", "amr_queue_capacity", "amr_active_missions",
             "amr_deadlock_active", "amr_gateway_requests_total"):
    check(name in sample_names, f"包含指标 {name}")

# 2.5 无 DispatcherState 时退化为 0（而不是缺样本或崩溃）
gauge_values = {}
for line in lines:
    match = SAMPLE_RE.match(line)
    if match and match.group(1) in ("amr_queue_size", "amr_active_missions"):
        gauge_values[match.group(1)] = float(match.group(3))
check(gauge_values.get("amr_queue_size") == 0.0,
      f"无状态时 amr_queue_size 应为 0，实际 {gauge_values.get('amr_queue_size')}")
check(gauge_values.get("amr_active_missions") == 0.0,
      f"无状态时 amr_active_missions 应为 0，实际 {gauge_values.get('amr_active_missions')}")

# 2.6 无受管车时不得产生空族（只有 HELP/TYPE 没有样本）
check("amr_vehicle_online" not in sample_names and "amr_vehicle_online" not in helped,
      "无受管车时不产生 amr_vehicle_online 空族")

# 2.7 网关自身计数自证：前面的 401 与本次 200 都要被记到
code, _, body2 = get("/metrics", metrics_token)
counts = {}
for line in body2.split("\n"):
    match = re.match(r'^amr_gateway_requests_total\{code="([0-9]{3})"\} (\S+)$', line)
    if match:
        counts[match.group(1)] = float(match.group(2))
check(counts.get("401", 0) >= 3,
      f'amr_gateway_requests_total{{code="401"}} 应 >= 3，实际 {counts.get("401")}')
check(counts.get("200", 0) >= 1,
      f'amr_gateway_requests_total{{code="200"}} 应 >= 1，实际 {counts.get("200")}')

if failures:
    print(f"\n[METRICS-FAIL] {len(failures)} 条断言失败")
    sys.exit(1)
print("\n[METRICS-OK] 全部断言通过")
PY

# 只用 -i 传 stdin（不给 -t，避免非交互环境下要 TTY）
docker exec -i "${CONTAINER}" python3 - "${METRICS_PORT}" "${METRICS_TOKEN}" \
    < "${ASSERT_SCRIPT}" || { echo "[METRICS-FAIL] /metrics 断言失败（见上方输出）" >&2; exit 1; }

# ---- 3. 进程仍在（抓取不应打死网关）----
docker exec "${CONTAINER}" bash -c \
    "pgrep -f 'rest_gateway.*port:=${METRICS_PORT}' >/dev/null" \
    || { echo "[METRICS-FAIL] /metrics 抓取后网关进程已退出" >&2; exit 1; }
echo "[Metrics] 抓取后网关进程存活 (PASS)"

echo ""
echo "======================================================================"
echo " /metrics 端点验收通过（域 ${METRICS_DOMAIN_ID} / 端口 ${METRICS_PORT}）"
echo "   - 口令隔离：/metrics 用 metrics_token，业务端点用 auth_token"
echo "   - 格式合法：# HELP / # TYPE 齐全，样本行全部可解析，无科学计数法"
echo "   - 无状态退化：未收到 DispatcherState 时调度指标为 0，且不产生空族"
echo "   - 自证计数：amr_gateway_requests_total 覆盖 401 与 200"
echo "======================================================================"
exit 0
