#!/usr/bin/env bash
# 开发机侧部署脚本：构建镜像 → 压缩直传服务器 → 由服务器从 origin 拉取代码 → 远端拉起
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
#
# 选项
# ----
#   --no-build         跳过镜像构建，直接传输本地已有的 $AMR_IMAGE
#   --image-tag TAG    指定镜像 tag（默认 amr-dispatcher:jazzy）
#   --repo-dir DIR     服务器上的部署目录（默认 /home/ubuntu/amr_dispatcher）
#   --swap-gb N        远端 swapfile 大小（默认 4，传 0 表示不创建）
#   --allow-unpushed   不校验本地提交是否已推送（默认会校验并拒绝部署未推送的提交）
#   -h, --help         显示帮助
#
# 环境变量
# ----
#   AMR_SERVER            目标服务器（必需），如 ubuntu@1.2.3.4
#   AMR_IMAGE             镜像 tag，默认 amr-dispatcher:jazzy
#   AMR_BRANCH            部署分支，默认 main
#   AMR_BUILD_HTTP_PROXY  构建期 HTTP 代理（仅在构建阶段 apt 需要时使用）
#   AMR_BUILD_HTTPS_PROXY 构建期 HTTPS 代理

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

IMAGE_TAG="${AMR_IMAGE:-amr-dispatcher:jazzy}"
REPO_DIR="/home/ubuntu/amr_dispatcher"
SWAP_GB=4
DO_BUILD=1
ALLOW_UNPUSHED="${AMR_ALLOW_UNPUSHED:-0}"

SERVER="${AMR_SERVER:-}"

while [ "$#" -gt 0 ]; do
  case "$1" in
    --no-build)         DO_BUILD=0; shift ;;
    --image-tag)        IMAGE_TAG="$2"; shift 2 ;;
    --repo-dir)         REPO_DIR="$2"; shift 2 ;;
    --swap-gb)          SWAP_GB="$2"; shift 2 ;;
    --allow-unpushed)   ALLOW_UNPUSHED=1; shift ;;
    -h|--help)          sed -n '2,28p' "$0"; exit 0 ;;
    -*)                 echo "未知选项: $1" >&2; exit 2 ;;
    *)                  SERVER="$1"; shift ;;
  esac
done

log()  { echo -e "\n=== $* ==="; }
warn() { echo "[WARN] $*" >&2; }
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

# ------------------------------------------------- 代码同步（经 git，非 tar）
# 为什么不用 tar 同步代码
# ----------------------
# 早先的实现用 `tar | ssh | tar x` 把整个仓库（含 .git/）推送到服务器。问题在于
# 服务器原有的 .git/config 会被开发机版本覆盖，一次造成两处回归：
#   1. 上游跟踪丢失 —— 开发机 .git/config 里没有 [branch "main"] 段，
#      覆盖后服务器 git pull 报 "There is no tracking information for the current branch"
#   2. 提交身份被改 —— 节区 [user] 变成了开发机的 tanqf，覆盖服务器全局的 dd2006tqf
#
# 改为：部署脚本只负责「构建镜像 + 传输镜像」，代码由服务器自己从 origin 同步
# （server_setup.sh 用 git fetch + reset --hard，见该脚本内说明）。
# 这样服务器保留自己的 .git/config（含上游跟踪与凭据），且代码的唯一来源是远端仓库。

BRANCH="${AMR_BRANCH:-main}"
if [ "$ALLOW_UNPUSHED" != "1" ]; then
  log "校验本地提交已推送到 origin/$BRANCH"
  if git rev-parse --verify -q "origin/$BRANCH" >/dev/null 2>&1; then
    if git merge-base --is-ancestor HEAD "origin/$BRANCH"; then
      echo "已推送: $(git rev-parse --short HEAD)"
    else
      die "本地 HEAD ($(git rev-parse --short HEAD)) 尚未推送到 origin/$BRANCH。

服务器将从这个远端分支拉取代码，未推送的提交不会被部署。
请先推送，或加 --allow-unpushed（注意：本地未推送的提交不会被部署，且服务器会因
检测到本地改动而中止——见 server_setup.sh 的说明）。"
    fi
  else
    warn "本地没有 origin/$BRANCH 引用，跳过校验（请确认服务器能取到相同代码）"
  fi
fi

# ------------------------------------------------------------ 远端启动
# git 身份与 safe.directory 由 server_setup.sh 负责，此处不再重复配置。
# server_setup.sh 会自行 git fetch + reset --hard 到 origin/$BRANCH。
log "在服务器上拉起服务（含从 origin/$BRANCH 同步代码）"
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
