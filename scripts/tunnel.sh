#!/usr/bin/env bash
# 建立到服务器的 SSH 隧道，把服务器的 127.0.0.1:8080 映射到本机 8080。
#
# 背景
# ----
# 服务器上 rest_gateway 绑定 0.0.0.0:8080（rest_gateway.cpp:106 硬编码 INADDR_ANY，
# 且无 bind 地址参数），应用本身也没有任何认证。docker-compose.server.yml 已把容器
# 端口限制为 "127.0.0.1:8080:8080"，因此公网无法直连，必须经隧道访问。
#
# 用法
# ----
#   AMR_SERVER=ubuntu@1.2.3.4 ./scripts/tunnel.sh
#   AMR_SERVER=ubuntu@1.2.3.4 ./scripts/tunnel.sh -p 9090   # 本机改监听 9090
#
# 隧道建立后：浏览器打开 http://127.0.0.1:8080/
# 按 Ctrl+C 断开。

set -euo pipefail

SERVER="${AMR_SERVER:-}"
LOCAL_PORT=8080

while [ "$#" -gt 0 ]; do
  case "$1" in
    -p|--local-port) LOCAL_PORT="$2"; shift 2 ;;
    -h|--help)       sed -n '2,18p' "$0"; exit 0 ;;
    *)               SERVER="$1"; shift ;;
  esac
done

[ -n "$SERVER" ] || { echo "未指定服务器。用法: AMR_SERVER=ubuntu@<ip> $0" >&2; exit 2; }

# 端口占用检查
if command -v ss >/dev/null 2>&1 && ss -tln 2>/dev/null | grep -q ":${LOCAL_PORT} "; then
  echo "[FAIL] 本机端口 ${LOCAL_PORT} 已被占用。用 -p <其它端口> 指定。" >&2
  exit 1
fi

echo "============================================================"
echo "  SSH 隧道: 本机 127.0.0.1:${LOCAL_PORT}  ->  ${SERVER}:127.0.0.1:8080"
echo "  隧道建立后，浏览器打开:  http://127.0.0.1:${LOCAL_PORT}/"
echo "  按 Ctrl+C 断开"
echo "============================================================"

exec ssh -N \
  -o ExitOnForwardFailure=yes \
  -o ServerAliveInterval=30 \
  -L "127.0.0.1:${LOCAL_PORT}:127.0.0.1:8080" \
  "$SERVER"
