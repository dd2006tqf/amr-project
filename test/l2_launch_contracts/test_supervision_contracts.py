#!/usr/bin/env python3
"""
L2 契约测试：进程守护与重生后重新提拉（launch_testing 真断言）。

要锁定的因果链（**不是**"进程死了会重启"这么一句口号）：

  1. `respawn=True` 的 `LifecycleNode` 被 SIGKILL 后，launch 会起一个新进程；
  2. 新进程回到 `unconfigured`——launch 的 `EmitEvent(CONFIGURE)` 与
     `OnStateTransition(inactive→ACTIVATE)` 只在**首次启动**时触发过一次，
     重生不会再触发（这是 launch 的既有行为，不是缺陷）；
  3. 因此"自愈"真正依赖的是 `lifecycle_coordinator_node` 的
     `auto_bringup=true` 周期提拉：unconfigured → inactive → active；
  4. `dispatcher_node` 的服务注册在 `on_configure` 而不是构造函数里，
     所以第 3 步成立之后服务会**重新出现**——否则网关会永久 503。

这四条里任何一条不成立，生产的"进程活着但功能没了"就会重演：
容器 `Up 8 days`、节点死 8 天、网关照样回 200。本测试在**不依赖 Docker**
的前提下把这条链子钉死。

域名：默认 94（`AMR_L2_SUPERVISION_DOMAIN_ID` 覆盖）。与 91 域的生命周期
契约测试分开，避免两套 launch 夹具的节点互相看见。

运行时依赖：launch_ros / launch_testing / rclpy，不依赖 map_server、MQTT
或任何文件系统状态。
"""

import os
import signal
import tempfile
import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
import rclpy
from rclpy.node import Node

from amr_dispatcher_interfaces.srv import GetSystemReadiness, ListStations


TEST_DOMAIN_ID = os.environ.get("AMR_L2_SUPERVISION_DOMAIN_ID", "94")

# 重生后重新提拉的预算：respawn_delay(1.0) + 协调器一拍(0.2) + configure 的
# 文件 IO/建模时间。给到 45s 是为了在慢 CI 上不抖动，而不是掩盖时序问题——
# 判据是"最终到了 active"，不是"多快"。
RECOVERY_BUDGET_SEC = 45.0

# 夹具站点目录：**必须显式注入**。dispatcher 的默认参数是相对路径
# `config/stations.yaml`，launch_testing 的工作目录是 build/<pkg>，
# 那里没有 config/ —— 目录加载失败时 dispatcher 仍会 active，
# 但 /v2/list_stations 会返回 "station catalog not loaded"，
# 于是"服务重建"的断言其实什么都没验证到（本测试先踩过这个坑）。
STATION_CATALOG = """version:
  map_hash: "l2_supervision_fixture_v1"
  graph_version: "v1"
stations:
  - id: "dock"
    x: 0.0
    y: 0.0
    yaw: 0.0
    type: "charger"
  - id: "pick"
    x: 4.0
    y: 0.0
    yaw: 0.0
    type: "pickup"
  - id: "drop"
    x: 8.0
    y: 0.0
    yaw: 0.0
    type: "dropoff"
road_nodes:
  - id: "r1"
    x: 2.0
    y: 0.0
    yaw: 0.0
  - id: "r2"
    x: 6.0
    y: 0.0
    yaw: 0.0
edges:
  - from: "dock"
    to: "r1"
    distance_m: 2.0
    bidirectional: true
    enabled: true
    polyline:
      - [0.0, 0.0]
      - [2.0, 0.0]
  - from: "r1"
    to: "pick"
    distance_m: 2.0
    bidirectional: true
    enabled: true
    polyline:
      - [2.0, 0.0]
      - [4.0, 0.0]
  - from: "pick"
    to: "r2"
    distance_m: 2.0
    bidirectional: true
    enabled: true
    polyline:
      - [4.0, 0.0]
      - [6.0, 0.0]
  - from: "r2"
    to: "drop"
    distance_m: 2.0
    bidirectional: true
    enabled: true
    polyline:
      - [6.0, 0.0]
      - [8.0, 0.0]
"""


def _dispatcher_pids(domain_id):
    """列出本域内 dispatcher_node 的进程号。

    只按 cmdline 匹配会误伤宿主机上其它域的 dispatcher（91 域契约测试、
    各 e2e 夹具都在跑同一个可执行文件），所以必须再看该进程的
    ROS_DOMAIN_ID 环境变量。
    """
    marker = b"lib/amr_dispatcher_ros/dispatcher_node"
    want_env = f"ROS_DOMAIN_ID={domain_id}".encode()
    pids = []
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        try:
            with open(f"/proc/{entry}/cmdline", "rb") as f:
                cmdline = f.read().split(b"\0")
            if not any(marker in part for part in cmdline):
                continue
            with open(f"/proc/{entry}/environ", "rb") as f:
                environ = f.read().split(b"\0")
        except OSError:
            continue
        if want_env in environ:
            pids.append(int(entry))
    return sorted(pids)


HEARTBEAT_FEEDER = """#!/usr/bin/env python3
# 给 safety_gate 的看门狗喂两路上游心跳。仅用于夹具：生产里这两路由
# chassis_driver_node / path_tracker_node 发布。
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


def main():
    rclpy.init()
    node = Node("l2_supervision_heartbeat_feeder")
    pubs = [
        node.create_publisher(Bool, "/safety/heartbeat/chassis_driver", 10),
        node.create_publisher(Bool, "/safety/heartbeat/path_tracker", 10),
    ]
    msg = Bool()
    msg.data = True

    def tick():
        for pub in pubs:
            pub.publish(msg)

    node.create_timer(0.2, tick)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # launch 关停时发 SIGTERM，rclpy 的信号处理器已经 rcl_shutdown 过一次；
        # 再调一次会抛 RCLError 并以 1 退出，让 post_shutdown 的退出码断言
        # 变成噪声。这里显式容忍"已关闭"。
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
"""


def _write_heartbeat_feeder():
    handle = tempfile.NamedTemporaryFile(
        mode="w", suffix="_heartbeat.py", prefix="l2_supervision_", delete=False)
    handle.write(HEARTBEAT_FEEDER)
    handle.close()
    return handle.name


def _write_station_catalog():
    handle = tempfile.NamedTemporaryFile(
        mode="w", suffix="_stations.yaml", prefix="l2_supervision_", delete=False)
    handle.write(STATION_CATALOG)
    handle.close()
    return handle.name


def generate_test_description():
    stations_path = _write_station_catalog()

    # auto_bringup=True 是本用例的被测对象（重生后的提拉全靠它）。
    # check_period_ms 调小到 200ms，让"重新提拉"在预算内可观测。
    coordinator_node = launch_ros.actions.Node(
        package="amr_dispatcher_ros",
        executable="lifecycle_coordinator_node",
        name="lifecycle_coordinator_node",
        parameters=[{"auto_bringup": True, "check_period_ms": 200}],
        output="screen",
        additional_env={"ROS_DOMAIN_ID": TEST_DOMAIN_ID},
    )
    # 安全门必须在场：协调器的就绪判定是
    # "dispatcher active AND 安全状态新鲜且健康"，缺了它永远 DEGRADED。
    safety_gate_node = launch_ros.actions.Node(
        package="amr_dispatcher_ros",
        executable="safety_gate_node",
        name="safety_gate_node",
        output="screen",
        additional_env={"ROS_DOMAIN_ID": TEST_DOMAIN_ID},
    )
    # 上游心跳：safety_gate 的 500ms 看门狗盯着 chassis_driver / path_tracker
    # 两路心跳。本夹具没有真实底盘与循迹节点，不喂心跳的话安全门按设计判上游
    # 失联、降级到 CRITICAL，/system/ready 永远不可能 READY。补的是"上游在位"，
    # 不是放宽判据（生产里这两路心跳由对应节点发布）。
    heartbeat_path = _write_heartbeat_feeder()
    heartbeat_fixture = launch.actions.ExecuteProcess(
        cmd=["python3", "-u", heartbeat_path],
        name="l2_supervision_heartbeat_fixture",
        output="screen",
        env={**os.environ, "ROS_DOMAIN_ID": TEST_DOMAIN_ID},
    )
    # 被守护的节点。respawn_delay 1.0s：短到测试跑得动，
    # 长到足以让"新 PID ≠ 旧 PID"这件事没有歧义。
    dispatcher_node = launch_ros.actions.Node(
        package="amr_dispatcher_ros",
        executable="dispatcher_node",
        name="dispatcher_node",
        parameters=[{
            "max_active_missions": 1,
            "stations_file": stations_path,
        }],
        output="screen",
        additional_env={"ROS_DOMAIN_ID": TEST_DOMAIN_ID},
        respawn=True,
        respawn_delay=1.0,
    )

    return (
        launch.LaunchDescription(
            [
                coordinator_node,
                safety_gate_node,
                heartbeat_fixture,
                dispatcher_node,
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {
            "coordinator": coordinator_node,
            "safety_gate": safety_gate_node,
            "dispatcher": dispatcher_node,
            "heartbeat": heartbeat_fixture,
        },
    )


class TestSupervisionContract(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        os.environ["ROS_DOMAIN_ID"] = TEST_DOMAIN_ID
        rclpy.init()
        cls.node = Node("l2_supervision_contract_probe")
        cls.ready_cli = cls.node.create_client(GetSystemReadiness, "/system/ready")
        cls.dispatcher_torn_down = False
        cls.respawn_evidence = None

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    # ---- 工具 -------------------------------------------------------------

    def _call_ready(self, timeout_sec=5.0):
        if not self.ready_cli.wait_for_service(timeout_sec=timeout_sec):
            return None
        future = self.ready_cli.call_async(GetSystemReadiness.Request())
        deadline = time.monotonic() + timeout_sec
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return future.result() if future.done() else None

    def _wait_for_ready(self, timeout_sec, label):
        deadline = time.monotonic() + timeout_sec
        last = None
        while time.monotonic() < deadline:
            resp = self._call_ready(timeout_sec=2.0)
            if resp is not None:
                last = resp
                if resp.ready and resp.state == "READY":
                    return last
            time.sleep(0.2)
        self.fail(f"{label}: {timeout_sec}s 内未达到 READY（最后状态 "
                  f"{getattr(last, 'state', 'N/A')!r}, "
                  f"message={getattr(last, 'message', 'N/A')!r}）")

    def _kill_dispatcher_and_wait_recovery(self):
        """SIGKILL dispatcher，然后等到"新 PID + 重新 active + READY"。

        结果缓存在类上：多个测试方法共享同一次破坏-恢复过程，
        避免重复 kill（重复 kill 会让断言之间的依赖变得难以推理）。
        """
        if self.__class__.respawn_evidence is not None:
            return self.__class__.respawn_evidence

        before = _dispatcher_pids(TEST_DOMAIN_ID)
        self.assertTrue(before, "夹具里没有找到 dispatcher_node 进程（域过滤后为空）")
        old_pid = before[0]
        os.kill(old_pid, signal.SIGKILL)

        # 1) 等到一个**不同**的 PID 出现：证明 launch 的 respawn 生效。
        # 2) 同时必须观测到"未就绪"：证明协调器没有在骗人（如果它一直报
        #    READY，生产上"节点死了但门禁说 OK"的故障模式根本没被覆盖）。
        #    注意这是**因果**断言而非时序断言：未就绪可以出现在新进程起来
        #    之前（旧服务消失）或之后（新节点还是 unconfigured），两者都合法。
        #    只有"从未未就绪、直接一路 READY"才是缺陷。
        new_pid = None
        saw_not_ready = False
        last_ready = None
        deadline = time.monotonic() + RECOVERY_BUDGET_SEC
        while time.monotonic() < deadline:
            pids = _dispatcher_pids(TEST_DOMAIN_ID)
            fresh = [p for p in pids if p != old_pid]
            if fresh and new_pid is None:
                new_pid = fresh[0]
            resp = self._call_ready(timeout_sec=1.0)
            if resp is not None:
                last_ready = resp
                if not resp.ready:
                    saw_not_ready = True
                elif new_pid is not None and saw_not_ready:
                    break
            time.sleep(0.2)

        self.assertIsNotNone(new_pid, f"{RECOVERY_BUDGET_SEC}s 内没有出现新的 dispatcher_node 进程")
        self.assertNotEqual(new_pid, old_pid)
        self.assertTrue(
            saw_not_ready,
            "dispatcher 被 SIGKILL 后协调器从未报未就绪（就绪判定没有跟随实际状态）。"
            f"最后状态={getattr(last_ready, 'state', 'N/A')!r} "
            f"ready={getattr(last_ready, 'ready', 'N/A')!r}")

        # 3) 重新提拉：新进程最终必须回到 READY
        final = self._wait_for_ready(RECOVERY_BUDGET_SEC, "重生后重新提拉")

        self.__class__.respawn_evidence = {
            "old_pid": old_pid,
            "new_pid": new_pid,
            "final": final,
        }
        return self.__class__.respawn_evidence

    # ---- 断言 -------------------------------------------------------------

    def test_bringup_reaches_ready(self):
        """auto_bringup=true 时冷启动必须最终达到 READY（后续断言的基线）。"""
        resp = self._wait_for_ready(60.0, "冷启动")
        self.assertIn("dispatcher_node", list(resp.active_nodes),
                      f"active_nodes 未包含 dispatcher_node: {list(resp.active_nodes)}")

    def test_respawned_dispatcher_is_repromoted(self):
        """核心契约：kill -9 之后新进程出现，且被协调器重新提到 active。"""
        evidence = self._kill_dispatcher_and_wait_recovery()
        self.assertNotEqual(evidence["new_pid"], evidence["old_pid"])
        self.assertTrue(evidence["final"].ready,
                        "重生后的 dispatcher 没有回到 READY（协调器未重新提拉？）")
        self.assertEqual(evidence["final"].state, "READY")

    def test_services_reregistered_after_respawn(self):
        """重生并重新 activate 之后，on_configure 里注册的服务必须重新可用。

        这条是"进程活着但功能没了"的直接反面证据：如果服务注册写在构造函数里
        而 lifecycle 又拦着未激活的服务，或者 on_configure 没被再执行一次，
        网关就会永久 503 —— 而容器仍显示 Up。
        """
        self._kill_dispatcher_and_wait_recovery()
        cli = self.node.create_client(ListStations, "/v2/list_stations")
        try:
            self.assertTrue(cli.wait_for_service(timeout_sec=30.0),
                            "重生后 /v2/list_stations 未重新出现（on_configure 未重跑？）")
            future = cli.call_async(ListStations.Request())
            deadline = time.monotonic() + 15.0
            while not future.done() and time.monotonic() < deadline:
                rclpy.spin_once(self.node, timeout_sec=0.05)
            self.assertTrue(future.done(), "重生后 /v2/list_stations 调用超时")
            self.assertTrue(future.result().success,
                            f"重生后 /v2/list_stations 返回失败: {future.result().message!r}")
        finally:
            self.node.destroy_client(cli)


@launch_testing.post_shutdown_test()
class TestSupervisionShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        """夹具里的节点要么以 0 退出，要么被按预期杀掉（-9 是被测动作）。"""
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -9, 137])
