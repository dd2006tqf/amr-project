#!/usr/bin/env bash
# ==============================================================================
# check_rest_gateway_hardening.sh - REST 网关健壮性与安全边界验收
#
# 目标：把"已复现的网关缺陷"变成可复现的回归断言。旧网关在这台机器上实测：
#   * 发一个 `Content-Length: abc` -> terminate called after throwing an
#     instance of 'std::invalid_argument' -> 整个网关进程 abort
#   * `prio=notanumber` -> 同上
#   * 建连后不发数据 -> 后续所有请求超时（单线程同步 accept 被卡死）
#
# 本脚本只依赖 python3（标准库 socket），不需要 netcat / curl 之外的额外工具；
# 全部请求走裸 socket，能构造 curl 不会发的畸形报文。
#
# 环境隔离：ROS_DOMAIN_ID=86 / REST 18096，与生产（域 42 / 8080）及其它 e2e
#           （77/78/80/82/84）完全隔离。不启动 dispatcher：本脚本验的是
#           **网关自身的边界**，后端服务缺失时应回 503 而不是崩溃。
#
# 退出码：0 = 全部断言通过；非 0 = 有断言失败（现场保留在 /tmp 便于排查）
# ==============================================================================
set -eo pipefail
set -E
trap 'echo "[HARD-FATAL] line=${LINENO} status=$? cmd=${BASH_COMMAND}" >&2' ERR

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

CONTAINER="${AMR_HARD_CONTAINER:-amr_dispatcher}"
HARD_DOMAIN_ID="${AMR_HARD_DOMAIN_ID:-86}"
HARD_PORT="${AMR_HARD_PORT:-18096}"
HARD_AUTH_PORT="${AMR_HARD_AUTH_PORT:-18097}"
CONTAINER_WORKDIR="${AMR_HARD_WORKDIR:-/workspace}"
STAMP_DIR="${CONTAINER_WORKDIR}/.hard_stamp"

HARD_FAILED=0

for tool in docker python3; do
    command -v "${tool}" >/dev/null 2>&1 || {
        echo "[HARD-FAIL] 宿主机缺少必需工具: ${tool}" >&2
        exit 2
    }
done
docker ps --format '{{.Names}}' | grep -qx "${CONTAINER}" || {
    echo "[HARD-FAIL] 容器 ${CONTAINER} 未运行" >&2
    exit 2
}

fail() {
    echo ""
    echo "[HARD-FAIL] 验收失败: $1"
    echo "--- [LOG TAIL: gateway (no-auth)] ---"
    docker exec "${CONTAINER}" bash -c "tail -n 40 '${STAMP_DIR}/gateway.log' 2>/dev/null" || echo "(no log)"
    echo "--- [LOG TAIL: gateway (auth)] ---"
    docker exec "${CONTAINER}" bash -c "tail -n 20 '${STAMP_DIR}/gateway_auth.log' 2>/dev/null" || echo "(no log)"
    HARD_FAILED=1
    exit 1
}

cleanup() {
    local exit_code=$?
    echo ""
    echo "[HARD] 正在清理..."
    docker exec "${CONTAINER}" bash -c "
        pkill -9 -f '[r]est_gateway.*port:=${HARD_PORT}' 2>/dev/null || true
        pkill -9 -f '[r]est_gateway.*port:=${HARD_AUTH_PORT}' 2>/dev/null || true
        for envf in /proc/[0-9]*/environ; do
            pid=\"\${envf#/proc/}\"; pid=\"\${pid%/environ}\"
            [ -r \"\${envf}\" ] || continue
            if tr '\\0' '\\n' 2>/dev/null < \"\${envf}\" | grep -qx 'ROS_DOMAIN_ID=${HARD_DOMAIN_ID}'; then
                kill -9 \"\${pid}\" 2>/dev/null || true
            fi
        done
        ROS_DOMAIN_ID=${HARD_DOMAIN_ID} bash -c 'source /opt/ros/jazzy/setup.bash >/dev/null 2>&1; ros2 daemon stop >/dev/null 2>&1' || true
    " >/dev/null 2>&1 || true
    if [ ${exit_code} -eq 0 ] && [ ${HARD_FAILED} -eq 0 ]; then
        docker exec "${CONTAINER}" bash -c "rm -rf '${STAMP_DIR}'" >/dev/null 2>&1 || true
        echo "[HARD] 全部通过，已清理临时现场"
    else
        echo "[HARD][WARN] 验收未全部成功，现场保留在容器内: ${STAMP_DIR}"
    fi
    exit ${exit_code}
}
trap cleanup EXIT INT TERM

# ---------------------------------------------------------------------------
# 在容器内拉起被测网关（两个实例：默认无鉴权 / 配置了 auth_token）
# ---------------------------------------------------------------------------
docker exec "${CONTAINER}" bash -c "rm -rf '${STAMP_DIR}' && mkdir -p '${STAMP_DIR}'"

cat > /tmp/hard_start.sh <<INNER
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source "${CONTAINER_WORKDIR}/install/setup.bash"
export ROS_DOMAIN_ID=${HARD_DOMAIN_ID}
STAMP="${STAMP_DIR}"

ros2 run amr_dispatcher_tools rest_gateway --ros-args \\
    -p port:=${HARD_PORT} \\
    -p html_path:=${CONTAINER_WORKDIR}/tools/operator_console.html \\
    > "\${STAMP}/gateway.log" 2>&1 &

ros2 run amr_dispatcher_tools rest_gateway --ros-args \\
    -p port:=${HARD_AUTH_PORT} \\
    -p auth_token:=hardening-secret \\
    -p html_path:=${CONTAINER_WORKDIR}/tools/operator_console.html \\
    > "\${STAMP}/gateway_auth.log" 2>&1 &

for port in ${HARD_PORT} ${HARD_AUTH_PORT}; do
    ready=0
    for ((i = 0; i < 200; i++)); do
        if bash -c "cat < /dev/null > /dev/tcp/127.0.0.1/\${port}" 2>/dev/null; then
            ready=1
            break
        fi
        sleep 0.1
    done
    [ "\${ready}" -eq 1 ] && echo "READY \${port}" >> "\${STAMP}/READY"
done
sleep 3600
INNER
docker cp /tmp/hard_start.sh "${CONTAINER}:${STAMP_DIR}/start.sh" >/dev/null
docker exec -d "${CONTAINER}" bash "${STAMP_DIR}/start.sh" >/dev/null 2>&1
echo "[HARD] 容器内网关已拉起（域 ${HARD_DOMAIN_ID} / 端口 ${HARD_PORT} + ${HARD_AUTH_PORT}）"

READY=0
for ((i = 0; i < 200; i++)); do
    if docker exec "${CONTAINER}" bash -c "test \$(wc -l < '${STAMP_DIR}/READY' 2>/dev/null || echo 0) -ge 2" 2>/dev/null; then
        READY=1
        break
    fi
    sleep 0.2
done
[ "${READY}" -eq 1 ] || fail "网关未在预期时间内就绪"

# ---------------------------------------------------------------------------
# 断言驱动：python3 裸 socket，逐条打印 PASS/FAIL
# ---------------------------------------------------------------------------
cat > /tmp/hard_assert.py <<'PY'
import socket
import sys
import time

HOST = "127.0.0.1"
PORT = int(sys.argv[1])
AUTH_PORT = int(sys.argv[2])
TOKEN = "hardening-secret"

failures = []


def check(name, ok, detail=""):
    print(("  [PASS] " if ok else "  [FAIL] ") + name + ((" :: " + detail) if detail else ""))
    if not ok:
        failures.append(name)


def raw(payload, port=None, timeout=8.0, read_all=True):
    """发原始字节，返回 (status_line, body)。连接被重置返回 ('', '')。"""
    port = port or PORT
    try:
        s = socket.create_connection((HOST, port), timeout=timeout)
    except OSError as e:
        return "", "connect-failed: %s" % e
    try:
        s.settimeout(timeout)
        s.sendall(payload if isinstance(payload, bytes) else payload.encode())
        chunks = []
        try:
            while True:
                data = s.recv(65536)
                if not data:
                    break
                chunks.append(data)
                if not read_all:
                    break
        except socket.timeout:
            pass
        raw_resp = b"".join(chunks).decode("utf-8", "replace")
        head, _, body = raw_resp.partition("\r\n\r\n")
        status = head.split("\r\n")[0] if head else ""
        return status, body
    except OSError as e:
        return "", "io-error: %s" % e
    finally:
        try:
            s.close()
        except OSError:
            pass


def status_code(status_line):
    parts = status_line.split()
    return int(parts[1]) if len(parts) >= 2 and parts[1].isdigit() else 0


def alive(port=None):
    status, _ = raw("GET /api/v1/state HTTP/1.1\r\nHost: x\r\nContent-Length: 0\r\n\r\n",
                    port=port)
    return status_code(status) > 0


print("[1] 畸形 Content-Length 不能让进程崩溃")
status, body = raw("GET /api/v1/state HTTP/1.1\r\nHost: x\r\nContent-Length: abc\r\n\r\n")
check("Content-Length: abc -> 400", status_code(status) == 400, status or body)
check("进程存活（畸形请求后仍能服务）", alive())

print("[2] 非数字 prio 不能让进程崩溃")
status, body = raw(
    "POST /api/v1/orders?id=m1&from=A&to=B&prio=notanumber HTTP/1.1\r\n"
    "Host: x\r\nContent-Length: 0\r\n\r\n")
check("prio=notanumber -> 400", status_code(status) == 400, status or body)
check("进程存活（畸形 prio 后仍能服务）", alive())

print("[3] 负数 Content-Length 一律拒绝（旧实现会转成极大值无限阻塞）")
status, body = raw("GET / HTTP/1.1\r\nHost: x\r\nContent-Length: -1\r\n\r\n")
check("Content-Length: -1 -> 400", status_code(status) == 400, status or body)
check("进程存活", alive())

print("[4] 请求体上限")
big = b"POST /api/v1/orders?id=m1 HTTP/1.1\r\nHost: x\r\nContent-Length: %d\r\n\r\n" % (
    2 * 1024 * 1024) + b"x" * (2 * 1024 * 1024)
status, body = raw(big, timeout=15)
check("2 MiB 请求 -> 413", status_code(status) == 413, status or body)
check("进程存活", alive())

print("[5] 精确路由（旧实现用子串匹配，ordersXYZ 会命中订单路由）")
status, _ = raw("POST /api/v1/ordersXYZ HTTP/1.1\r\nHost: x\r\nContent-Length: 0\r\n\r\n")
check("/api/v1/ordersXYZ -> 404", status_code(status) == 404, status)
status, _ = raw("POST /api/v1/orders/extra?id=m1 HTTP/1.1\r\nHost: x\r\nContent-Length: 0\r\n\r\n")
check("/api/v1/orders/extra -> 404", status_code(status) == 404, status)

print("[6] 分块传输拒绝（网关不实现 chunked，猜长度会读到连接耗尽）")
status, _ = raw("POST /api/v1/orders?id=m1 HTTP/1.1\r\nHost: x\r\nTransfer-Encoding: chunked\r\n\r\n0\r\n\r\n")
check("Transfer-Encoding: chunked -> 400", status_code(status) == 400, status)

print("[7] 静默连接不得卡死网关")
silent = socket.create_connection((HOST, PORT), timeout=5)
try:
    started = time.time()
    status, _ = raw("GET /api/v1/state HTTP/1.1\r\nHost: x\r\nContent-Length: 0\r\n\r\n",
                    timeout=6)
    elapsed = time.time() - started
    check("静默连接存在期间另一连接正常响应", status_code(status) == 200,
          "%s (%.2fs)" % (status, elapsed))
    check("另一连接未被长时间阻塞（<3s）", elapsed < 3.0, "%.2fs" % elapsed)
finally:
    silent.close()

print("[7b] 迟迟不发完整请求的连接：超时后静默关闭，不得凭空变成 404")
# 真实事故：accept 快车道把"半截请求"以 kTimeout 交给 worker，worker 只等满
# 自己的 5s 后拿回 kTimeout，而 HandleClient 只识别 kClosed
# -> 一个**全空**的请求一路走到路由兜底，返回 404 {"error":"not_found"}。
# 浏览器侧看起来就是"控制台偶发变成一页 not_found"。
# 正确语义：超时 = 对端没交出请求，等同于 kClosed，静默关闭。
late = socket.create_connection((HOST, PORT), timeout=10)
try:
    late.settimeout(20)
    started = time.time()
    late.sendall(b"GET / HTTP/1.1\r\nHost: x\r\n")  # 故意不发结尾的空行
    received = b""
    try:
        while True:
            data = late.recv(65536)
            if not data:
                break
            received += data
    except socket.timeout:
        pass
    elapsed = time.time() - started
finally:
    late.close()
check("半截请求在连接超时后被静默关闭（无任何响应字节）", received == b"",
      "%d bytes, %.1fs" % (len(received), elapsed))
check("半截请求不得返回 404 not_found", b"not_found" not in received,
      received[:80].decode("utf-8", "replace"))

print("[8] 鉴权（默认关闭 -> 独立实例配置 auth_token）")
status, _ = raw("GET /api/v1/state HTTP/1.1\r\nHost: x\r\nContent-Length: 0\r\n\r\n",
                port=AUTH_PORT)
check("未配 token 的实例不需要鉴权",
      status_code(raw("GET /api/v1/state HTTP/1.1\r\nHost: x\r\nContent-Length: 0\r\n\r\n")[0]) == 200)
check("配了 token 的实例：无 Authorization -> 401", status_code(status) == 401, status)
status, _ = raw("GET /api/v1/state HTTP/1.1\r\nHost: x\r\nAuthorization: Bearer wrong\r\n"
                "Content-Length: 0\r\n\r\n", port=AUTH_PORT)
check("错误 token -> 401", status_code(status) == 401, status)
status, _ = raw("GET /api/v1/state HTTP/1.1\r\nHost: x\r\nAuthorization: Bearer %s\r\n"
                "Content-Length: 0\r\n\r\n" % TOKEN, port=AUTH_PORT)
check("正确 token -> 200", status_code(status) == 200, status)
status, _ = raw("GET / HTTP/1.1\r\nHost: x\r\nContent-Length: 0\r\n\r\n", port=AUTH_PORT)
check("操作台页面本身免鉴权（可不带 token 打开）", status_code(status) == 200, status)

print("[9] 响应体必须是合法 JSON（字段含引号/换行时不能拼坏）")
import json
status, body = raw("GET /api/v1/state HTTP/1.1\r\nHost: x\r\nContent-Length: 0\r\n\r\n")
try:
    json.loads(body)
    ok = True
    detail = ""
except Exception as e:  # noqa: BLE001
    ok = False
    detail = str(e)
check("GET /api/v1/state 返回合法 JSON", ok, detail)

print("[10] 非法 JSON 请求体不崩溃")
status, _ = raw("POST /api/v1/force_clear_link HTTP/1.1\r\nHost: x\r\nContent-Length: 5\r\n\r\n{{{{{")
check("畸形请求体 -> 400（缺字段）", status_code(status) == 400, status)
check("进程存活", alive())

print("[11] 幂等键：后端失败时不得被「吃掉」（旧实现先插入后调用）")
# 本实例没有 dispatcher 后端：两次同 key 请求都应回 503。
# 关键断言是**第二次不是 duplicate** —— 说明首次失败已回滚该 key。
req = ("POST /api/v1/orders?id=idem_m1&from=A&to=B&prio=5&idempotency_key=key-retry-1 "
       "HTTP/1.1\r\nHost: x\r\nContent-Length: 0\r\n\r\n")
status1, body1 = raw(req)
status2, body2 = raw(req)
check("首次请求因后端不可用回 503", status_code(status1) == 503, status1 or body1)
check("同 key 重试仍回 503（未被误判为 duplicate）",
      status_code(status2) == 503, status2 + " " + body2)
check("重试响应不含 idempotent_duplicate", "idempotent_duplicate" not in body2, body2)

print("")
if failures:
    print("[HARD-ASSERT] 失败 %d 条: %s" % (len(failures), ", ".join(failures)))
    sys.exit(1)
print("[HARD-ASSERT] 全部断言通过")
PY

docker cp /tmp/hard_assert.py "${CONTAINER}:${STAMP_DIR}/assert.py" >/dev/null
if ! docker exec "${CONTAINER}" python3 "${STAMP_DIR}/assert.py" "${HARD_PORT}" "${HARD_AUTH_PORT}"; then
    fail "网关加固断言未全部通过"
fi

echo ""
echo "======================================================================"
echo " [PASS] REST 网关健壮性验收全部通过"
echo "======================================================================"
exit 0
