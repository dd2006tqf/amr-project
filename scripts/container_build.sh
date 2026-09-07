#!/usr/bin/env bash
set -e

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
ROOT_DIR="$DIR/.."

echo "=== Building ROS 2 Workspace inside Container ==="
docker run --rm \
  --net=host \
  -v "${ROOT_DIR}:/workspace/amr_dispatcher" \
  -w /workspace/amr_dispatcher \
  amr-dispatcher:latest \
  bash -c "source /opt/ros/jazzy/setup.bash && \
           colcon build --base-paths src --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"

echo "=== ROS 2 Build Completed Successfully! ==="
