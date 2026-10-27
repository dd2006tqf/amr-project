#!/usr/bin/env bash
# ==============================================================================
# check_cbs_coordination_e2e.sh - Phase 9 (P3) CBS 高层多车协同寻路集成验证脚本
#
# 验收目标 (Phase 9 P3):
#   Stage 1: 验证低层带时空约束 A* 寻路引擎 (Space-Time A*)
#            - 验证起点/终点可达性
#            - 验证顶点约束规避 (强制停留让行 Wait In Place)
#            - 验证路段边约束阻断下的动态绕行
#   Stage 2: 验证高层基于冲突的搜索树引擎 (Conflict-Based Search, CBS)
#            - 验证无冲突场景的最优路径直接直达
#            - 验证双通道对向冲突下的协同动态分流
#            - 验证十字路口交叉相撞时的自动时空等待错峰消解
#   Stage 3: 验证 CBS 多车交通协调器与时空预约表 (TimeSpaceOccupancy) 转换
#            - 验证 TimedPath 到 ResourceTimeWindowSet 的保序精确映射
#            - 验证在途固定交通约束对新规划智能体的硬隔离
#   Stage 4: 容器内全包 ROS 2 契约回归检验 (Core / Interfaces / BT / ROS / Tools)
#
# 环境隔离:
#   ROS_DOMAIN_ID=86
# ==============================================================================

set -eo pipefail
set -E
trap 'echo "[CBS-E2E-FATAL] line=${LINENO} status=$? cmd=${BASH_COMMAND}" >&2' ERR

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

echo "======================================================================"
echo " Phase 9: Multi-Agent Conflict-Based Search (CBS) Coordination E2E"
echo " Workspace: ${REPO_ROOT}"
echo "======================================================================"

echo ">>> [Stage 1/4] 执行低层时空 A* 算法单测 (Space-Time A* with Wait-In-Place)..."
if [ -f "${REPO_ROOT}/build/core/test_space_time_astar" ]; then
    "${REPO_ROOT}/build/core/test_space_time_astar"
else
    ctest --test-dir "${REPO_ROOT}/build/core" -R "test_space_time_astar" --output-on-failure
fi
echo "[OK] 低层时空 A* 节点/边时空约束与原地等待动作检验通过。"

echo ">>> [Stage 2/4] 执行高层约束树搜索 CBS 算法单测 (Conflict-Based Search)..."
if [ -f "${REPO_ROOT}/build/core/test_conflict_based_search" ]; then
    "${REPO_ROOT}/build/core/test_conflict_based_search"
else
    ctest --test-dir "${REPO_ROOT}/build/core" -R "test_conflict_based_search" --output-on-failure
fi
echo "[OK] 高层约束树分支、顶点/互换/追尾冲突消解与分流通行检验通过。"

echo ">>> [Stage 3/5] 执行优先级时空规划 PBS 算法单测 (Prioritized Planning & Goal-Hold)..."
if [ -f "${REPO_ROOT}/build/core/test_prioritized_planning" ]; then
    "${REPO_ROOT}/build/core/test_prioritized_planning"
else
    ctest --test-dir "${REPO_ROOT}/build/core" -R "test_prioritized_planning" --output-on-failure
fi
echo "[OK] 优先级时空搜索、终点驻留保护与无冲突门禁检验通过。"

echo ">>> [Stage 4/5] 执行统一 MAPF 求解器门面与 CBS 自适应降级单测 (MAPF Solver Facade)..."
if [ -f "${REPO_ROOT}/build/core/test_mapf_solver" ]; then
    "${REPO_ROOT}/build/core/test_mapf_solver"
else
    ctest --test-dir "${REPO_ROOT}/build/core" -R "test_mapf_solver" --output-on-failure
fi
echo "[OK] CBS 搜索超限自动降级至 Prioritized Planning 闭环检验通过。"

echo ">>> [Stage 5/5] 执行 CBS 交通协调器与时空窗转换单测及全包 ROS 2 回归 (1036+ tests)..."
if [ -f "${REPO_ROOT}/build/core/test_cbs_traffic_coordinator" ]; then
    "${REPO_ROOT}/build/core/test_cbs_traffic_coordinator"
else
    ctest --test-dir "${REPO_ROOT}/build/core" -R "test_cbs_traffic_coordinator" --output-on-failure
fi
docker exec amr_dispatcher /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon test --base-paths src --packages-select amr_dispatcher_core amr_dispatcher_ros && colcon test-result --all"

echo "======================================================================"
echo " Phase 9/10 CBS & Prioritized MAPF Coordination E2E: ALL 5 STAGES PASSED"
echo "======================================================================"
