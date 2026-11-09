#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPORT_FILE="${WORKSPACE_DIR}/test_report_matrix.json"

START_TIME=$(date +%s)

echo "================================================================="
echo "       AMR Dispatcher 工业级系统基础设施与混沌演练流水线         "
echo "================================================================="

# ---------------------------------------------------------------------------
# 阶段状态一律由实际执行结果生成：PASSED / FAILED / SKIPPED。
#   PASSED  = 阶段执行成功
#   FAILED  = 阶段执行失败（计入失败数，脚本最终以非零退出）
#   SKIPPED = 本机不具备执行条件（不计入失败，但如实写入报告与 verdict）
# 矩阵中不再写入任何硬编码的通过结论；verdict 只基于实际执行的阶段。
# ---------------------------------------------------------------------------
CORE_STATUS="SKIPPED"
BENCH_STATUS="SKIPPED"
BT_STATUS="SKIPPED"
CHAOS_DEADLOCK_STATUS="SKIPPED"
CHAOS_WATCHDOG_STATUS="SKIPPED"
CHAOS_FAILOVER_STATUS="SKIPPED"
FAILED_STAGES=0

# record_status <目标变量名> <PASSED|FAILED>
record_status() {
  local -n _status_ref="$1"
  _status_ref="$2"
  if [ "$2" = "FAILED" ]; then
    FAILED_STAGES=$((FAILED_STAGES + 1))
  fi
}

# count_suites <目录...>：统计测试源码中唯一 TEST/TEST_F 套件名数量（无匹配时输出 0）
count_suites() {
  {
    grep -rhoE 'TEST(_F)?[[:space:]]*\([[:space:]]*[A-Za-z0-9_]+' "$@" 2>/dev/null \
      | sed -E 's/.*\([[:space:]]*//' \
      | sort -u \
      | wc -l \
      | tr -d '[:space:]'
  } || true
}

# count_ctest_cases <构建目录>：读取 ctest 注册的用例总数（仅列举不执行）
count_ctest_cases() {
  {
    ctest --test-dir "$1" -N 2>/dev/null \
      | sed -n 's/.*Total Tests: \([0-9][0-9]*\).*/\1/p' \
      | tail -1
  } || true
}

# json_number <值>：空值降级为 JSON null
json_number() {
  if [ -n "$1" ]; then printf '%s' "$1"; else printf 'null'; fi
}

# ---------------------------------------------------------------------------
# L1 核心层：纯 C++ 编译 + GoogleTest
# ---------------------------------------------------------------------------
echo ""
echo "[L1 核心层] 编译纯 C++ 核心库并执行 GoogleTest 单元测试 (零 ROS 依赖)..."
if "${WORKSPACE_DIR}/scripts/build.sh"; then
  record_status CORE_STATUS PASSED
else
  echo "[L1 核心层] FAILED：构建或单测未通过" >&2
  record_status CORE_STATUS FAILED
fi

CORE_TEST_CASES=""
CORE_TEST_SUITES=""
if [ "$CORE_STATUS" = "PASSED" ]; then
  CORE_TEST_CASES="$(count_ctest_cases "${WORKSPACE_DIR}/build/core")"
  CORE_TEST_SUITES="$(count_suites "${WORKSPACE_DIR}/test/unit" "${WORKSPACE_DIR}/test/integration")"
fi

# ---------------------------------------------------------------------------
# L1 压测层：基准压测（依赖 L1 构建产物）
# ---------------------------------------------------------------------------
BENCH_LOG="$(mktemp)"
# M6：脚本早退路径也要清理 benchmark 临时文件
trap 'rm -f "${BENCH_LOG}"' EXIT
BENCH_QUEUE_NOTE="not measured (benchmark stage not executed)"
BENCH_LOGGER_NOTE="not measured (benchmark stage not executed)"
if [ "$CORE_STATUS" = "PASSED" ]; then
  echo ""
  echo "[L1 压测层] 执行核心库极限基准压测 (任务队列并发 & 异步低阻塞日志)..."
  if "${WORKSPACE_DIR}/scripts/benchmark.sh" 2>&1 | tee "${BENCH_LOG}"; then
    record_status BENCH_STATUS PASSED
  else
    echo "[L1 压测层] FAILED：基准压测未通过" >&2
    record_status BENCH_STATUS FAILED
  fi
  bench_queue_push="$(sed -n 's/.*Push throughput: \([0-9.]*\) ops\/sec.*/\1/p' "${BENCH_LOG}" | head -1)"
  bench_queue_pop="$(sed -n 's/.*Pop throughput:[[:space:]]*\([0-9.]*\) ops\/sec.*/\1/p' "${BENCH_LOG}" | head -1)"
  bench_logger="$(sed -n 's/.*Enqueue throughput: \([0-9.]*\) ops\/sec.*/\1/p' "${BENCH_LOG}" | head -1)"
  if [ "$BENCH_STATUS" = "PASSED" ]; then
    if [ -n "$bench_queue_push" ] && [ -n "$bench_queue_pop" ]; then
      BENCH_QUEUE_NOTE="push ${bench_queue_push} ops/sec, pop ${bench_queue_pop} ops/sec (measured this run)"
    fi
    if [ -n "$bench_logger" ]; then
      BENCH_LOGGER_NOTE="enqueue ${bench_logger} ops/sec (measured this run)"
    fi
  fi
else
  echo ""
  echo "[L1 压测层] SKIPPED：依赖 L1 核心库构建产物"
fi
rm -f "${BENCH_LOG}"

# ---------------------------------------------------------------------------
# L1 行为树层：amr_dispatcher_bt（ament/colcon 产物；纯宿主机无此产物则 SKIPPED）
# ---------------------------------------------------------------------------
BT_BUILD_DIR="${WORKSPACE_DIR}/build/amr_dispatcher_bt"
BT_TEST_CASES=""
BT_TEST_SUITES=""
if [ -d "${BT_BUILD_DIR}" ]; then
  # 构建产物可能来自 Docker 容器（CTestTestfile 内记录 /workspace 绝对路径），
  # 宿主机上这些路径不存在，ctest 必然报错——此时应判 SKIPPED 而非 FAILED。
  BT_FOREIGN_BUILD_DIR="$(sed -n 's/^# Build directory: //p' "${BT_BUILD_DIR}/CTestTestfile.cmake" 2>/dev/null | head -1)"
  if [ -n "${BT_FOREIGN_BUILD_DIR}" ] && [ ! -d "${BT_FOREIGN_BUILD_DIR}" ]; then
    echo ""
    echo "[L1 行为树层] SKIPPED：BT 构建产物来自容器（${BT_FOREIGN_BUILD_DIR} 在宿主机不存在），宿主机无法执行；请在容器/CI 内运行"
  else
    echo ""
    echo "[L1 行为树层] 执行 amr_dispatcher_bt 行为树单元测试 (BehaviorTree.CPP v4 + GTest)..."
    if ctest --test-dir "${BT_BUILD_DIR}" --output-on-failure; then
      record_status BT_STATUS PASSED
      BT_TEST_CASES="$(count_ctest_cases "${BT_BUILD_DIR}")"
      BT_TEST_SUITES="$(count_suites "${WORKSPACE_DIR}/src/amr_dispatcher_bt/test")"
    else
      echo "[L1 行为树层] FAILED：BT ctest 未通过" >&2
      record_status BT_STATUS FAILED
    fi
  fi
else
  echo ""
  echo "[L1 行为树层] SKIPPED：build/amr_dispatcher_bt 不存在（BT 包需 colcon/ament 构建，本环境未构建）"
fi

# ---------------------------------------------------------------------------
# L3 场景层：混沌演练（依赖 L1 构建产物）
# ---------------------------------------------------------------------------
if [ "$CORE_STATUS" = "PASSED" ]; then
  echo ""
  echo "[L3 场景层: 混沌演练 1] 执行拓扑路网死锁检测与恢复演练..."
  if "${WORKSPACE_DIR}/scripts/check_deadlock_resolution.sh"; then
    record_status CHAOS_DEADLOCK_STATUS PASSED
  else
    echo "[L3 混沌演练 1] FAILED：死锁检测与恢复演练未通过" >&2
    record_status CHAOS_DEADLOCK_STATUS FAILED
  fi

  echo ""
  echo "[L3 场景层: 混沌演练 2] 执行软看门狗心跳丢失与安全门熔断制动演练..."
  if "${WORKSPACE_DIR}/scripts/check_watchdog_estop.sh"; then
    record_status CHAOS_WATCHDOG_STATUS PASSED
  else
    echo "[L3 混沌演练 2] FAILED：看门狗/急停演练未通过" >&2
    record_status CHAOS_WATCHDOG_STATUS FAILED
  fi

  echo ""
  echo "[L3 场景层: 混沌演练 3] 执行底层通信丢包监控与三级容灾降级演练..."
  if "${WORKSPACE_DIR}/scripts/check_hardware_failover.sh"; then
    record_status CHAOS_FAILOVER_STATUS PASSED
  else
    echo "[L3 混沌演练 3] FAILED：硬件降级演练未通过" >&2
    record_status CHAOS_FAILOVER_STATUS FAILED
  fi
else
  echo ""
  echo "[L3 场景层] SKIPPED：依赖 L1 核心库构建产物"
  CHAOS_DEADLOCK_STATUS="SKIPPED (依赖 L1)"
  CHAOS_WATCHDOG_STATUS="SKIPPED (依赖 L1)"
  CHAOS_FAILOVER_STATUS="SKIPPED (依赖 L1)"
fi

END_TIME=$(date +%s)
DURATION=$((END_TIME - START_TIME))

# ---------------------------------------------------------------------------
# verdict：只基于实际执行的阶段
#   FAILED                        —— 任一执行阶段失败
#   PASSED_WITH_SKIPPED_STAGES    —— 已执行阶段全过，但存在 SKIPPED 阶段
#   ALL_PASSED                    —— 全部阶段执行且全部通过
# ---------------------------------------------------------------------------
ANY_SKIPPED=0
for _s in "$CORE_STATUS" "$BENCH_STATUS" "$BT_STATUS" \
          "$CHAOS_DEADLOCK_STATUS" "$CHAOS_WATCHDOG_STATUS" "$CHAOS_FAILOVER_STATUS"; do
  case "$_s" in
    SKIPPED*) ANY_SKIPPED=1 ;;
  esac
done

if [ "${FAILED_STAGES}" -gt 0 ]; then
  VERDICT="FAILED"
elif [ "${ANY_SKIPPED}" -eq 1 ]; then
  VERDICT="PASSED_WITH_SKIPPED_STAGES"
else
  VERDICT="ALL_PASSED"
fi

# 生成格式化测试矩阵交付报告（全部字段来自本次实际执行结果）
cat <<EOF > "${REPORT_FILE}"
{
  "project": "amr_dispatcher",
  "version": "1.2.0",
  "timestamp": "$(date -u +"%Y-%m-%dT%H:%M:%SZ")",
  "duration_seconds": ${DURATION},
  "testing_matrix": {
    "L1_core_unit_tests": {
      "status": "${CORE_STATUS}",
      "test_suites": $(json_number "${CORE_TEST_SUITES}"),
      "test_cases": $(json_number "${CORE_TEST_CASES}"),
      "dependencies": "Zero-ROS (Pure C++17/20 STL, GTest)",
      "note": "suites 源码实测统计, cases 取 ctest 实测注册数; 数字随测试增补而变化"
    },
    "L1_behavior_tree_tests": {
      "status": "${BT_STATUS}",
      "test_suites": $(json_number "${BT_TEST_SUITES}"),
      "test_cases": $(json_number "${BT_TEST_CASES}"),
      "framework": "BehaviorTree.CPP v4 + GTest",
      "note": "仅在 build/amr_dispatcher_bt 存在(colcon 构建过)时执行, 否则 SKIPPED"
    },
    "L1_performance_benchmark": {
      "status": "${BENCH_STATUS}",
      "mission_queue_throughput": "${BENCH_QUEUE_NOTE}",
      "async_logger_throughput": "${BENCH_LOGGER_NOTE}"
    },
    "L2_node_contracts": {
      "status": "SKIPPED",
      "reason": "无自动化执行框架: test/l2_launch_contracts 仅为脚本样例, 未接入 ctest/launch_testing",
      "framework": "ROS 2 launch_testing & Lifecycle Coordinator Probes (规划中)",
      "contracts": [
        "/system/ready",
        "/system/healthy",
        "ExecuteMission.action",
        "12 Core Dispatcher Services (/v2/*)"
      ]
    },
    "L3_chaos_scenarios": {
      "deadlock_wfg_cycle_resolution": "${CHAOS_DEADLOCK_STATUS}",
      "watchdog_200ms_timeout_cutoff": "${CHAOS_WATCHDOG_STATUS}",
      "hardware_failover_3tier_degradation": "${CHAOS_FAILOVER_STATUS}"
    }
  },
  "verdict": "${VERDICT}"
}
EOF

echo ""
echo "================================================================="
if [ "${FAILED_STAGES}" -gt 0 ]; then
  echo "  [FAILED] ${FAILED_STAGES} 个阶段失败，verdict=${VERDICT}，详见上方日志。"
  echo "  📄 测试报告（含失败状态）已写入: ${REPORT_FILE}"
  echo "================================================================="
  exit 1
fi
echo "  [ALL EXECUTED STAGES PASSED] verdict=${VERDICT}（SKIPPED 阶段以报告为准）"
echo "  📄 测试报告已固化至: ${REPORT_FILE}"
echo "================================================================="
