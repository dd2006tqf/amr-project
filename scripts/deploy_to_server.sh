#!/usr/bin/env bash
# 开发机侧部署脚本：构建镜像 → 压缩直传服务器 → 同步代码 → 远端拉起
#
# 设计要点
# --------
# 1. 镜像通过 `docker save | gzip | ssh "gunzip | docker load"` 管道传输，
#    服务器上不留中转 tar（省约 2.5GB 磁盘）。
# 2. 服务器内存仅 3.6GiB，故镜像在开发机构建，服务器只负责运行。
# 3. 服务器 IP 不写死在本脚本中（仓库对外公开，避免暴露服务器地址）。
#
# 用法
# ----
#   AMR_SERVER=ubuntu@1.2.3.4 ./scripts/deploy_to_server.sh
#   AMR_SERVER=ubuntu@1.2.3.4 ./scripts/deploy_to_server.sh --no-build   # 复用已有镜像
#   AMR_SERVER=ubuntu@1.2.3.4 ./scripts/deploy_to_server.sh --skip-ssh-config
#
# 选项
# ----
#   --no-build         跳过镜像构建，直接传输本地已有的 $AMR_IMAGE
#   --image-tag TAG    指定镜像 tag（默认 amr-dispatcher:jazzy）
#   --repo-dir DIR     服务器上的部署目录（默认 /opt/amr_dispatcher）
#   --swap-gb N        远端 swapfile 大小（默认 4，传 0 表示不创建）
#   --skip-ssh-config  跳过远端 git 身份/仓库配置
#   -h, --help         显示帮助

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

IMAGE_TAG="${AMR_IMAGE:-amr-dispatcher:jazzy}"
REPO_DIR="/opt/amr_dispatcher"
SWAP_GB=4
DO_BUILD=1
SKIP_SSH_CONFIG=0

SERVER="${AMR_SERVER:-}"

while [ "$#" -gt 0 ]; do
  case "$1" in
    --no-build)         DO_BUILD=0; shift ;;
    --image-tag)        IMAGE_TAG="$2"; shift 2 ;;
    --repo-dir)         REPO_DIR="$2"; shift 2 ;;
    --swap-gb)          SWAP_GB="$2"; shift 2 ;;
    --skip-ssh-config)  SKIP_SSH_CONFIG=1; shift ;;
    -h|--help)          sed -n '2,25p' "$0"; exit 0 ;;
    -*)                 echo "未知选项: $1" >&2; exit 2 ;;
    *)                  SERVER="$1"; shift ;;
  esac
done

log()  { echo -e "\n=== $* ==="; }
die()  { echo "[FAIL] $*" >&2; exit 1; }

[ -n "$SERVER" ] || die "未指定服务器。请用 AMR_SERVER=ubuntu@<ip> 或位置参数传入。"
[ -f "${REPO_ROOT}/docker/Dockerfile.jazzy" ] || die "找不到 docker/Dockerfile.jazzy，请在仓库根目录执行。"

cd "$REPO_ROOT"

# ------------------------------------------------------------ 前置检查
log "前置检查"
SSH_OPTS=(-o BatchMode=yes -o ConnectTimeout=15)
ssh "${SSH_OPTS[@]}" "$SERVER" 'true' 2>/dev/null \
  || die "无法免密 SSH 到 $SERVER。请先确认公钥已安装（或改用带密码的方式手动执行）。"
echo "SSH 免密登录 OK: $SERVER"

ssh "${SSH_OPTS[@]}" "$SERVER" 'docker info >/dev/null 2>&1' \
  || die "$SERVER 上 docker 不可用（或当前用户不在 docker 组）。"
echo "远端 docker OK"

if [ "$DO_BUILD" = "1" ]; then
  docker image inspect ros:jazzy-ros-base >/dev/null 2>&1 \
    || die "本机缺少基础镜像 ros:jazzy-ros-base，请先 docker pull ros:jazzy-ros-base"
fi

# ------------------------------------------------------------ 构建镜像
if [ "$DO_BUILD" = "1" ]; then
  log "构建镜像 $IMAGE_TAG"
  # 构建不访问网络装包（依赖已在 base 中），故无需传代理 build-arg。
  # 若你的网络环境需要代理，可导出 BUILDKIT_PROXY 或直接改用 docker build --build-arg。
  PROXY_ARGS=()
  if [ -n "${AMR_BUILD_HTTP_PROXY:-}" ]; then
    PROXY_ARGS+=(--build-arg "HTTP_PROXY=${AMR_BUILD_HTTP_PROXY}")
  fi
  if [ -n "${AMR_BUILD_HTTPS_PROXY:-}" ]; then
    PROXY_ARGS+=(--build-arg "HTTPS_PROXY=${AMR_BUILD_HTTPS_PROXY}")
  fi

  docker build \
    -f docker/Dockerfile.jazzy \
    -t "$IMAGE_TAG" \
    "${PROXY_ARGS[@]}" \
    .

  docker images --format '{{.Repository}}:{{.Tag}}  {{.Size}}' | grep -F "$IMAGE_TAG" || true
else
  log "跳过构建（--no-build）"
  docker image inspect "$IMAGE_TAG" >/dev/null 2>&1 || die "本机没有镜像 $IMAGE_TAG"
fi

# ------------------------------------------------------------ 传输镜像
log "传输镜像到 $SERVER（压缩管道直传，服务器不留中转文件）"
echo "提示：传输量约 1–2GB，耗时取决于上行带宽，请耐心等待。"
START=$(date +%s)
docker save "$IMAGE_TAG" | gzip -1 | \
  ssh "${SSH_OPTS[@]}" "$SERVER" 'gunzip | docker load'
END=$(date +%s)
echo "传输完成，用时 $((END - START)) 秒"

# ------------------------------------------------------------ 同步代码
log "同步代码到 $SERVER:$REPO_DIR"
# 含 .git（仅约 5MB），使远端成为可正常 git pull 的工作副本；
# 排除 415MB 构建产物与 3MB 简历 PDF。
tar czf - \
  --exclude=./build \
  --exclude=./install \
  --exclude=./log \
  --exclude='*.pdf' \
  --exclude=./.git/index.lock \
  -C "$REPO_ROOT" . | \
  ssh "${SSH_OPTS[@]}" "$SERVER" "sudo mkdir -p '$REPO_DIR' && sudo chown \$(id -u):\$(id -g) '$REPO_DIR' && tar xzf - -C '$REPO_DIR'"
echo "代码同步完成"

# ---------------------------------------------------- 远端 SSH 身份配置
if [ "$SKIP_SSH_CONFIG" = "0" ]; then
  log "配置远端 git 身份（便于日后直接在服务器上提交）"
  ssh "${SSH_OPTS[@]}" "$SERVER" bash -s <<'EOF'
set -e
cd /opt/amr_dispatcher
if [ -z "$(git config --global user.name || true)" ]; then
  git config --global user.name "dd2006tqf"
fi
if [ -z "$(git config --global user.email || true)" ]; then
  git config --global user.email "dd2006tqf@users.noreply.github.com"
fi
if ! git config --global --get-all safe.directory 2>/dev/null | grep -qx '/opt/amr_dispatcher'; then
  git config --global --add safe.directory /opt/amr_dispatcher
fi
echo "--- 远端 git 状态 ---"
git remote -v || true
git log --oneline -1 || true
EOF
fi

# ------------------------------------------------------------ 远端启动
log "在服务器上拉起服务"
ssh "${SSH_OPTS[@]}" "$SERVER" \
  "cd '$REPO_DIR' && chmod +x scripts/*.sh docker/*.sh && ./scripts/server_setup.sh --repo-dir '$REPO_DIR' --swap-gb $SWAP_GB"

log "完成"
cat <<EOF

后续操作：

  查看日志
    ssh $SERVER 'docker compose -f $REPO_DIR/docker/docker-compose.server.yml logs -f'

  建 SSH 隧道后访问 Web 控制台
    AMR_SERVER=$SERVER ./scripts/tunnel.sh
    浏览器打开 http://127.0.0.1:8080/

  健康检查
    ssh $SERVER 'docker exec -it amr_dispatcher bash -lc "source install/setup.bash && ros2 node list"'

  代码更新后重新部署（推送到 GitHub 后，服务器可直接拉取）
    ssh $SERVER 'cd $REPO_DIR && git pull'
    # 若改动了源码需重建镜像，则在本机重新执行本脚本

EOF
