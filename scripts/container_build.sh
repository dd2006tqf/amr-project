#!/usr/bin/env bash
# 在容器内构建整个 ROS 2 工作区（仅构建，不启动服务）。
#
# 与 docker/entrypoint.server.sh 的关系
# ------------------------------------
# 正式的部署流程由 entrypoint.server.sh 负责（按需构建 + 启动服务）。
# 本脚本用于「只想构建、不想起服务」的场景，例如本地验证 colcon 能否通过。
#
# 注意镜像 tag 与挂载点
# --------------------
# 镜像用 amr-dispatcher:jazzy（由 docker/Dockerfile.jazzy 构建）；
# 挂载点用 /workspace 并与工作目录保持一致 —— entrypoint 与
# rest_gateway / dispatcher_visualizer_node 的相对路径加载都依赖这个布局。
#
# 旧的 amr-dispatcher:latest / robot-amr:latest 是已废弃的 7GB 镜像，
# 对应的 docker/Dockerfile 与 docker/docker-compose.yml 已从仓库移除。

set -e

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
ROOT_DIR="$DIR/.."
IMAGE="${AMR_IMAGE:-amr-dispatcher:jazzy}"
WORKERS="${AMR_COLCON_WORKERS:-$(nproc)}"

if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
  echo "[FAIL] 镜像 $IMAGE 不存在。" >&2
  echo "       请先构建：docker build -f docker/Dockerfile.jazzy -t $IMAGE ." >&2
  exit 1
fi

echo "=== 在容器内构建 ROS 2 工作区（镜像 $IMAGE，并行度 $WORKERS） ==="
docker run --rm \
  -v "${ROOT_DIR}:/workspace" \
  -w /workspace \
  "$IMAGE" \
  bash -c "source /opt/ros/jazzy/setup.bash && \
           colcon build --base-paths src --symlink-install --parallel-workers ${WORKERS} \
                        --cmake-args -DCMAKE_BUILD_TYPE=Release"

echo "=== ROS 2 构建完成 ==="
