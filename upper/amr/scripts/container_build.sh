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

set -euo pipefail

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
ROOT_DIR="$DIR/.."
IMAGE="${AMR_IMAGE:-amr-dispatcher:jazzy}"

# 并发度按**内存**推导，不再用 $(nproc)：见 scripts/lib/build_workers.sh。
#
# 注意这里在**宿主机**上推导，而编译发生在容器里。这是刻意的：容器默认
# 不设内存上限（HostConfig.Memory=0），两者看到的是同一块物理内存；
# 而宿主机侧推导能在 `docker run` 之前就把值算好并写进命令行，不依赖
# 镜像里有没有脚本。若将来给容器加了 --memory，请同时把该限额也传给
# 这里的推导（否则宿主机会按物理内存算出偏大的并发）。
# ⚠️ lib/build_workers.sh 是**可执行脚本**（stdout 输出一个整数），不是
# shell 函数库。必须「执行」它并读 stdout；`source` 后调 $(build_workers)
# 会因函数不存在而得到空值 → CMAKE_BUILD_PARALLEL_LEVEL="" → 不限并发。
# 执行失败时回退到 1，绝不能把空值传下去（2026-11-09 修复）。
WORKERS="${AMR_COLCON_WORKERS:-}"
if [ -z "$WORKERS" ]; then
  WORKERS="$(bash "$DIR/lib/build_workers.sh" 2>/dev/null)" || WORKERS=""
fi
case "$WORKERS" in
  '' | *[!0-9]*) WORKERS=1 ;;
esac

# 两级并发都要限：colcon 的 --parallel-workers 只管包级，包内部的 make/ninja
# 仍按 CPU 数开 -j（详见 build_workers.sh 的说明）。
# 通过 -e 传进容器，容器内的 cmake 才能读到。
export CMAKE_BUILD_PARALLEL_LEVEL="$WORKERS"

if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
  echo "[FAIL] 镜像 $IMAGE 不存在。" >&2
  echo "       请先构建：docker build -f docker/Dockerfile.jazzy -t $IMAGE ." >&2
  exit 1
fi

echo "=== 在容器内构建 ROS 2 工作区（镜像 $IMAGE，并行度 $WORKERS） ==="
docker run --rm \
  -v "${ROOT_DIR}:/workspace" \
  -w /workspace \
  -e CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL}" \
  "$IMAGE" \
  bash -c "source /opt/ros/jazzy/setup.bash && \
           colcon build --base-paths src --symlink-install --parallel-workers ${WORKERS} \
                        --cmake-args -DCMAKE_BUILD_TYPE=Release"

echo "=== ROS 2 构建完成 ==="
