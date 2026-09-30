#!/usr/bin/env bash
# 服务器侧部署脚本（幂等，可重复执行）
#
# 职责
# ----
# 1. 按需创建 swapfile（服务器内存仅 3.6GiB，swap 原已用 90%）
# 2. 准备 /opt/amr_dispatcher：有则 git pull，无则 git clone
# 3. 用 docker/docker-compose.server.yml 拉起容器
# 4. 打印状态与 SSH 隧道命令
#
# 前提
# ----
# - 仓库中的部署文件已推送到远程；或用 scripts/deploy_to_server.sh 从开发机带过去
# - 镜像 amr-dispatcher:jazzy 已 docker load 完成
#
# 用法（服务器上，或由 deploy_to_server.sh 经 ssh 调用）：
#   ./scripts/server_setup.sh
#   ./scripts/server_setup.sh --swap-gb 0        # 不创建 swap
#   ./scripts/server_setup.sh --repo-dir /opt/amr_dispatcher

set -euo pipefail

REPO_URL="${AMR_REPO_URL:-https://github.com/dd2006tqf/amr-project.git}"
REPO_DIR="${AMR_REPO_DIR:-/opt/amr_dispatcher}"
IMAGE="${AMR_IMAGE:-amr-dispatcher:jazzy}"
COMPOSE_FILE="docker/docker-compose.server.yml"
SWAP_GB=4
SWAP_PATH="/swapfile.amr"

while [ "$#" -gt 0 ]; do
  case "$1" in
    --swap-gb)   SWAP_GB="$2"; shift 2 ;;
    --repo-dir)  REPO_DIR="$2"; shift 2 ;;
    --repo-url)  REPO_URL="$2"; shift 2 ;;
    -h|--help)   sed -n '2,20p' "$0"; exit 0 ;;
    *) echo "未知参数: $1" >&2; exit 2 ;;
  esac
done

log()  { echo -e "\n=== $* ==="; }
warn() { echo "[WARN] $*" >&2; }

# ----------------------------------------------------------------- swap
if [ "$SWAP_GB" -gt 0 ]; then
  log "检查 swap"
  if swapon --show=NAME --noheadings 2>/dev/null | grep -q "^${SWAP_PATH}$"; then
    echo "$SWAP_PATH 已启用，跳过"
  else
    free -h | grep -i swap || true
    AVAIL_GB=$(df --output=avail -BG / | tail -1 | tr -dc '0-9')
    if [ "$AVAIL_GB" -lt $((SWAP_GB + 3)) ]; then
      warn "磁盘可用 ${AVAIL_GB}G，不足以再分出 ${SWAP_GB}G swap，跳过"
    else
      echo "创建 ${SWAP_GB}G swapfile: $SWAP_PATH"
      sudo fallocate -l "${SWAP_GB}G" "$SWAP_PATH"
      sudo chmod 600 "$SWAP_PATH"
      sudo mkswap "$SWAP_PATH" >/dev/null
      sudo swapon "$SWAP_PATH"
      # 记录到 fstab 以便重启后仍生效（注释掉可改为纯临时）
      if ! grep -qF "$SWAP_PATH" /etc/fstab; then
        echo "$SWAP_PATH none swap sw 0 0" | sudo tee -a /etc/fstab >/dev/null
        echo "已写入 /etc/fstab"
      fi
      swapon --show
    fi
  fi
fi

# ------------------------------------------------------------- 代码仓库
log "准备代码仓库 $REPO_DIR"
if [ -d "$REPO_DIR/.git" ]; then
  echo "已存在 git 仓库，执行 pull"
  git -C "$REPO_DIR" pull --ff-only || warn "pull 失败（可能有本地改动），继续使用现有工作区"
else
  echo "克隆 $REPO_URL"
  sudo mkdir -p "$(dirname "$REPO_DIR")"
  sudo chown "$(id -u):$(id -g)" "$(dirname "$REPO_DIR")"
  git clone "$REPO_URL" "$REPO_DIR"
fi

cd "$REPO_DIR"

# ---------------------------------------------------------------- 镜像
log "检查镜像 $IMAGE"
if docker image inspect "$IMAGE" >/dev/null 2>&1; then
  docker images --format '{{.Repository}}:{{.Tag}}  {{.Size}}' | grep -F "$IMAGE" || true
else
  echo "[FAIL] 镜像 $IMAGE 不存在。" >&2
  echo "       请在开发机执行: ./scripts/deploy_to_server.sh <user>@<host>" >&2
  exit 1
fi

# -------------------------------------------------------------- compose
if [ ! -f "$COMPOSE_FILE" ]; then
  echo "[FAIL] 找不到 $COMPOSE_FILE" >&2
  echo "       若该文件尚未提交到远程，请从开发机用 deploy_to_server.sh 同步。" >&2
  exit 1
fi

log "启动服务"
docker compose -f "$COMPOSE_FILE" up -d

# ---------------------------------------------------------------- 状态
log "容器状态"
docker compose -f "$COMPOSE_FILE" ps

log "磁盘占用"
df -h / | tail -1
docker system df

cat <<EOF

============================================================
  部署完成
============================================================

查看日志：
  docker compose -f $COMPOSE_FILE logs -f

首次启动会自动 colcon 构建（并行度 2），约需 5-15 分钟。
构建完成后容器内会有 7 个节点。

健康检查：
  docker exec -it amr_dispatcher bash -lc 'source install/setup.bash && ros2 node list'
  docker exec -it amr_dispatcher bash -lc 'source install/setup.bash && ros2 service list | grep -c /v2/'

访问 Web 控制台（需在本机建隧道）：
  ssh -N -L 8080:127.0.0.1:8080 \$(whoami)@<server-ip>
  然后浏览器打开 http://127.0.0.1:8080/

EOF
