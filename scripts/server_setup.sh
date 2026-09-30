#!/usr/bin/env bash
# 服务器侧部署脚本（幂等，可重复执行）
#
# 职责
# ----
# 1. 按需创建 swapfile（服务器内存仅 3.6GiB，swap 原已用 90%）
# 2. 准备 /home/ubuntu/amr_dispatcher：有则 git pull，无则 git clone
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
#   ./scripts/server_setup.sh --repo-dir /home/ubuntu/amr_dispatcher

set -euo pipefail

REPO_URL="${AMR_REPO_URL:-https://github.com/dd2006tqf/amr-project.git}"
REPO_DIR="${AMR_REPO_DIR:-/home/ubuntu/amr_dispatcher}"
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
BRANCH="${AMR_BRANCH:-main}"

if [ -d "$REPO_DIR/.git" ]; then
  cd "$REPO_DIR"

  # 自愈上游跟踪。历史上用 tar 同步 .git/ 会把开发机的 .git/config 覆盖到服务器，
  # 而开发机的 config 里没有 [branch "main"] 段，导致服务器 git pull 报
  # "There is no tracking information for the current branch"。
  if [ "$(git config --get "branch.$BRANCH.remote" || true)" != "origin" ]; then
    echo "修复 $BRANCH 的上游跟踪 -> origin/$BRANCH"
    git config "branch.$BRANCH.remote" origin
    git config "branch.$BRANCH.merge" "refs/heads/$BRANCH"
  fi

  echo "拉取 origin/$BRANCH"
  git fetch origin "$BRANCH"

  # 保护本地开发成果：reset --hard 会丢弃工作区改动，也会丢弃【未被推送】的提交。
  # 实测确认：已 push 的提交在 reset --hard 后仍在（它们在 origin/main 上）；
  # 但仅 commit 未 push 的提交会从分支上消失（只能靠 git reflog 捞回）。
  # 所以这里必须显式拦住，而不是静默覆盖。
  if [ "${AMR_FORCE_SYNC:-0}" != "1" ]; then
    unpushed=$(git rev-list --count "origin/$BRANCH..HEAD" 2>/dev/null || echo 0)
    # 只统计【已跟踪文件】的改动。未跟踪文件不在 reset --hard 的清理范围内
    # （那需要 git clean），所以不该因为它们挡住部署 —— 例如容器写出的
    # __pycache__、或本地随手放的文件。脚本后面会单独清掉 __pycache__。
    dirty=$(git status --porcelain --untracked-files=no | wc -l)
    untracked=$(git status --porcelain --untracked-files=normal | grep -c '^??' || true)

    if [ "$unpushed" -gt 0 ] || [ "$dirty" -gt 0 ]; then
      cat >&2 <<ERR

[FAIL] $REPO_DIR 有未同步到 origin/$BRANCH 的本地内容，已中止以免丢失：

        未推送的提交: $unpushed 个
        已跟踪文件的改动: $dirty 个

        git reset --hard 会丢弃这两类内容。请先处理：

          先推送（推荐）:
            cd $REPO_DIR && git add -A && git commit -m "..." && git push origin $BRANCH

          或先备份到别处:
            cd $REPO_DIR && git stash push -u -m "部署前备份"

          确认要放弃这些改动（危险，只能靠 git reflog 找回）:
            AMR_FORCE_SYNC=1 $0
ERR
      if [ "$unpushed" -gt 0 ]; then
        echo "" >&2
        echo "      未推送的提交如下：" >&2
        git log --oneline "origin/$BRANCH..HEAD" | sed 's/^/        /' >&2
      fi
      if [ "$dirty" -gt 0 ]; then
        echo "" >&2
        echo "      改动的文件如下：" >&2
        git status --porcelain --untracked-files=no | sed 's/^/        /' >&2
      fi
      exit 1
    fi

    # 未跟踪文件不影响同步，仅提示
    if [ "$untracked" -gt 0 ]; then
      echo "[INFO] 有 $untracked 个未跟踪文件，不受 reset --hard 影响，保留不动"
    fi
  else
    echo "[WARN] AMR_FORCE_SYNC=1，将丢弃本地未同步的提交与改动"
    echo "       如需事后找回: git reflog  （未被 gc 的提交可从 reflog 恢复）"
  fi

  # 用 reset --hard 而非 pull：部署目标是让工作区严格等于远端，
  # 同时能覆盖同步过程可能残留的改动（如历史 tar 同步留下的差异）。
  git checkout -f -q "$BRANCH" 2>/dev/null || true
  git reset --hard FETCH_HEAD

  # 清理容器以 root 身份写出的 .pyc。容器内 Python 启动 launch 文件时会在挂载的
  # 仓库里生成 root 所有的 __pycache__，既会污染 git status 的整洁性，
  # 也会让基于 tar 的同步因 "Cannot utime: Operation not permitted" 失败。
  sudo find "$REPO_DIR" -name __pycache__ -type d -prune -exec rm -rf {} + 2>/dev/null || true

  echo "已同步到: $(git log --oneline -1)"
else
  echo "克隆 $REPO_URL"
  sudo mkdir -p "$(dirname "$REPO_DIR")"
  sudo chown "$(id -u):$(id -g)" "$(dirname "$REPO_DIR")"
  git clone -b "$BRANCH" "$REPO_URL" "$REPO_DIR"
  cd "$REPO_DIR"
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
