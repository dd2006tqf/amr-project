#!/usr/bin/env python3
"""
L2 契约测试：生命周期协调与就绪探针（launch_testing 真断言）。

此前本文件只有 `assertTrue(True)`——它永远通过，因此"契约"实际上没有任何
约束力。这里改成对**真实进程**做可证伪的断言：

  1. 协调器启动后 `/system/ready`、`/system/healthy` 两个服务可调用
  2. auto_bringup=false 时 dispatcher 不被提拉 -> `/system/ready` 报
     ready=false 且 state=INITIALIZING（门禁语义：未就绪不放行）
  3. `/system/ready_flag`（transient_local）发布布尔量，且当前值为 false
  4. 关停时所有进程以 0 退出

用**三个节点**（coordinator + dispatcher + safety_gate）而不是只有两个：
就绪判定是 `dispatcher active AND 安全门健康`，缺了 safety_gate 时协调器会
永久停在 `DEGRADED`（安全门从未上报），那既不是"未就绪门禁"也不是我们要
锁定的语义。三个节点一起起，未就绪状态就退化成确定性的 `INITIALIZING`。

本测试在**独立的 ROS_DOMAIN_ID（默认 91）**下运行：默认域 0 上可能残留
上一次运行的同名节点（`ros2 node list` 会看到旧的 dispatcher），lifecycle
调用被路由到旧节点会让结论不可信。

运行时依赖：launch_ros / launch_testing / rclpy（CI 的 ros:jazzy-ros-base 镜像
已包含）。不依赖 map_server、MQTT 或任何文件系统状态。
"""

import os
import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from amr_dispatcher_interfaces.srv import GetSystemReadiness
from std_msgs.msg import Bool


TEST_DOMAIN_ID = os.environ.get("AMR_L2_DOMAIN_ID", "91")


def _node(package, executable, name, parameters=None):
    return launch_ros.actions.Node(
        package=package,
        executable=executable,
        name=name,
        parameters=parameters or [],
        output="screen",
        # 每个节点显式带域 ID：launch_testing 的子进程不继承我们的环境改写，
        # 显式传参才能保证三个节点与探针在同一张图上。
        additional_env={"ROS_DOMAIN_ID": TEST_DOMAIN_ID},
    )


def generate_test_description():
    # auto_bringup=false：协调器只看不拉，dispatcher 停在 unconfigured，
    # 于是"未就绪"是确定性的（不依赖时序竞态）。
    coordinator_node = launch_ros.actions.Node(
        package="amr_dispatcher_ros",
        executable="lifecycle_coordinator_node",
        name="lifecycle_coordinator_node",
        parameters=[{"auto_bringup": False, "check_period_ms": 100}],
        output="screen",
        additional_env={"ROS_DOMAIN_ID": TEST_DOMAIN_ID},
    )
    dispatcher_node = _node(
        "amr_dispatcher_ros", "dispatcher_node", "dispatcher_node",
        [{"max_active_missions": 1}])
    # 安全门必须在场：缺了它协调器永远停在 DEGRADED（从未收到安全状态）
    safety_gate_node = _node(
        "amr_dispatcher_ros", "safety_gate_node", "safety_gate_node")

    return (
        launch.LaunchDescription(
            [coordinator_node, safety_gate_node, dispatcher_node,
             launch_testing.actions.ReadyToTest()]
        ),
        {"coordinator": coordinator_node, "dispatcher": dispatcher_node,
         "safety_gate": safety_gate_node},
    )


class TestLifecycleCoordinationContract(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        os.environ["ROS_DOMAIN_ID"] = TEST_DOMAIN_ID
        rclpy.init()
        cls.node = Node("l2_lifecycle_contract_probe")
        cls.ready_cli = cls.node.create_client(GetSystemReadiness, "/system/ready")
        cls.healthy_cli = cls.node.create_client(GetSystemReadiness, "/system/healthy")
        cls.ready_flags = []
        cls.node.create_subscription(
            Bool,
            "/system/ready_flag",
            lambda msg: cls.ready_flags.append(msg.data),
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _call(self, client, timeout_sec=10.0):
        self.assertTrue(
            client.wait_for_service(timeout_sec=timeout_sec),
            f"{client.srv_name} 服务未在 {timeout_sec}s 内出现",
        )
        future = client.call_async(GetSystemReadiness.Request())
        deadline = time.monotonic() + timeout_sec
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertTrue(future.done(), f"{client.srv_name} 调用超时")
        return future.result()

    def test_readiness_and_health_services_are_callable(self):
        """/system/ready 与 /system/healthy 必须真实存在且能在超时内返回响应。"""
        ready = self._call(self.ready_cli)
        self.assertIsNotNone(ready, "/system/ready 返回为空")
        healthy = self._call(self.healthy_cli)
        self.assertIsNotNone(healthy, "/system/healthy 返回为空")

    def test_not_ready_while_dispatcher_inactive(self):
        """auto_bringup=false 时 dispatcher 未激活 -> 门禁必须报未就绪。

        这是门禁语义的核心：只要 dispatcher 不是 active，上游就不该放行派单。
        """
        resp = self._call(self.ready_cli)
        self.assertFalse(resp.ready, "dispatcher 未激活时 /system/ready 不应为 ready")
        self.assertEqual(resp.state, "INITIALIZING",
                         f"未就绪时应为 INITIALIZING，实际 {resp.state!r}")
        self.assertIn("dispatcher_node", list(resp.pending_nodes),
                      f"pending_nodes 应列出 dispatcher_node，实际 {list(resp.pending_nodes)}")

    def test_ready_flag_topic_publishes_false(self):
        """ready_flag 是 transient_local：晚加入的订阅者也能拿到当前值。"""
        deadline = time.monotonic() + 10.0
        while not self.ready_flags and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertTrue(self.ready_flags, "/system/ready_flag 未收到任何消息")
        self.assertFalse(self.ready_flags[-1], "dispatcher 未激活时 ready_flag 应为 false")


@launch_testing.post_shutdown_test()
class TestLifecycleCoordinationShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        """两个节点都必须正常退出（夹具里没有预期会失败的节点）。"""
        launch_testing.asserts.assertExitCodes(proc_info)
