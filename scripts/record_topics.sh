#!/usr/bin/env bash
set -e

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
OUTPUT_BAG="${1:-/tmp/amr_dispatcher_diagnostics_bag}"

echo "=== AMR Dispatcher Diagnostic Topic Recorder ==="
echo "Recording topics to: $OUTPUT_BAG"

if ! command -v ros2 &> /dev/null; then
  echo "[ERROR] ros2 CLI not found. Please source your ROS 2 environment first."
  exit 1
fi

ros2 bag record -o "$OUTPUT_BAG" \
  /odom \
  /chassis/link_health \
  /dispatcher/state \
  /dispatcher/events \
  /cmd_vel_safe \
  /cmd_vel_raw \
  /safety/state \
  /tracking/error \
  /system/aggregated_link_health
