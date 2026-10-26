#!/usr/bin/env bash
# check_traffic_coordination_e2e.sh
# Phase 7 (P3): 运行时交通协调与在途死锁恢复集成验证脚本
# 隔离运行域: ROS_DOMAIN_ID=89, REST 18097, MQTT 18899
set -euo pipefail

DOMAIN_ID=89
REST_PORT=18097
MQTT_PORT=18899

echo "=== Phase 7: Runtime Traffic Coordination & Deadlock Recovery E2E Test ==="
echo "Domain: ${DOMAIN_ID}, REST: ${REST_PORT}, MQTT: ${MQTT_PORT}"

# 1. 验证编译产物与测试矩阵
echo "[1/3] Running host core traffic coordination unit tests..."
ctest --test-dir /home/ubuntu/个人学习/amr_dispatcher/build/core -R "Traffic|InFlight" --output-on-failure

echo "[2/3] Verifying container-level ROS contracts..."
docker exec amr_dispatcher /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon test --base-paths src --packages-select amr_dispatcher_core amr_dispatcher_ros && colcon test-result --all"

echo "[3/3] Asserting traffic coordination state & 2-phase reservation transaction invariants..."
# 针对两阶段置换的断言：
# (a) 成功绕行时：新锁必须被预约，旧未来锁必须被释放，当前安全节点锁必须严格保留
# (b) 预约冲突时：新路线不下发，旧锁严格保留，任务进入等待或重试状态
/home/ubuntu/个人学习/amr_dispatcher/build/core/test_in_flight_reroute_planning
/home/ubuntu/个人学习/amr_dispatcher/build/core/test_traffic_recovery_coordinator
/home/ubuntu/个人学习/amr_dispatcher/build/core/test_traffic_coordination_model

echo "=== Phase 7 Traffic Coordination E2E: ALL CHECKS PASSED ==="
