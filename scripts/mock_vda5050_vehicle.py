#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""mock_vda5050_vehicle.py — VDA 5050 虚拟小车模拟器 (Task 6)

用途
====
在没有真实 AGV 的情况下, 用一台纯软件虚拟小车完整走通 VDA 5050 联调链路:
  - 订阅 uagv/v2/{manufacturer}/{serial}/{order, instantActions} (QoS 1)
  - 校验并解析订单 JSON (nodes/edges/sequenceId/released), 拒绝非法订单
  - 沿边折线 (trajectory.controlPoints) 以可配置速度模拟行驶
  - 2Hz 上报 VDA 5050 state (headerId 自增, orderId, lastNodeId/lastNodeSequenceId,
    agvPosition x/y/theta, batteryState —— 满足 DeserializeVda5050State 契约)
  - 连接即发 retained ONLINE connection 心跳, 并配置 LWT=CONNECTIONBROKEN
    (kill -9 / --drop-after-n-seconds 触发 broker 侧断链语义)
  - released=false 制动守卫: 遇未放行的边立即停车等待, 收到同 orderId 且
    orderUpdateId 更大的更新订单后恢复行驶 (VDA 5050 更新语义)
  - 状态文件持久化: 位置/进度/当前订单落盘, 重启后从断点恢复 (断链重连场景)
  - --self-test: 不连 MQTT 的自检模式 (订单解析 + 折线步进 + released 制动等待),
    供 check_vda5050_e2e.sh 作为第 0 步先跑

依赖: python3-paho-mqtt (>=1.6, 旧式 Client API)。 --self-test 模式无第三方依赖。

用法示例:
  python3 mock_vda5050_vehicle.py --host 127.0.0.1 --port 18884 \
      --manufacturer RobotManufacturer --serial mock_amr_01 --speed 1.0
  python3 mock_vda5050_vehicle.py --self-test
"""

import argparse
import json
import math
import os
import signal
import sys
import time

PROTOCOL_VERSION = "2.0.0"          # v2 线上版本号
DEFAULT_STATE_HZ = 2.0              # state 上报频率 (任务契约: 2Hz)
DEFAULT_HEARTBEAT_SEC = 5.0         # connection ONLINE retained 心跳周期
STATE_SAVE_PERIOD_SEC = 0.5         # 状态文件落盘周期


# =============================================================================
# 订单解析与校验 (与 amr_dispatcher_core SerializeVda5050Order 输出对齐)
# =============================================================================
class OrderError(ValueError):
    """订单 JSON 不满足 VDA 5050 / 本模拟器契约。"""


def _require(cond, msg):
    if not cond:
        raise OrderError(msg)


def _as_number(val, what):
    _require(isinstance(val, (int, float)) and not isinstance(val, bool),
             f"{what} 必须为数字, 实际: {val!r}")
    return float(val)


def _as_int(val, what):
    _require(isinstance(val, int) and not isinstance(val, bool), f"{what} 必须为整数, 实际: {val!r}")
    return val


def _as_str(val, what):
    _require(isinstance(val, str) and val != "", f"{what} 必须为非空字符串, 实际: {val!r}")
    return val


def _as_bool(val, what):
    _require(isinstance(val, bool), f"{what} 必须为布尔, 实际: {val!r}")
    return val


def parse_order(raw):
    """校验并规范化订单 dict。非法时抛 OrderError, 合法时返回内部订单结构。

    内部结构: {
      "order_id": str, "order_update_id": int,
      "nodes": [{"node_id", "sequence_id", "released", "x", "y", "theta"} ...],
      "edges": [{"edge_id", "sequence_id", "start_node_id", "end_node_id", "released",
                 "polyline": [(x, y), ...]} ...],
    }
    契约: edges[i] 恰为 nodes[i] -> nodes[i+1]; 折线不足 2 点时退化为直线段。
    """
    _require(isinstance(raw, dict), "订单必须为 JSON 对象")
    order_id = _as_str(raw.get("orderId"), "orderId")
    update_id = _as_int(raw.get("orderUpdateId"), "orderUpdateId")
    _require(update_id >= 0, f"orderUpdateId 不得为负: {update_id}")

    raw_nodes = raw.get("nodes")
    _require(isinstance(raw_nodes, list) and len(raw_nodes) >= 1,
             f"nodes 必须为非空数组, 实际: {type(raw_nodes).__name__}")
    nodes = []
    seen_seq = set()
    prev_seq = -1
    for i, rn in enumerate(raw_nodes):
        _require(isinstance(rn, dict), f"nodes[{i}] 必须为对象")
        node_id = _as_str(rn.get("nodeId"), f"nodes[{i}].nodeId")
        seq = _as_int(rn.get("sequenceId"), f"nodes[{i}].sequenceId")
        _require(seq >= 0, f"nodes[{i}].sequenceId 不得为负: {seq}")
        _require(seq not in seen_seq, f"sequenceId 重复: {seq}")
        _require(seq > prev_seq, f"nodes sequenceId 必须严格递增: {prev_seq} -> {seq}")
        seen_seq.add(seq)
        prev_seq = seq
        released = _as_bool(rn.get("released"), f"nodes[{i}].released")
        pos = rn.get("nodePosition")
        _require(isinstance(pos, dict), f"nodes[{i}].nodePosition 必须为对象")
        x = _as_number(pos.get("x"), f"nodes[{i}].nodePosition.x")
        y = _as_number(pos.get("y"), f"nodes[{i}].nodePosition.y")
        theta = _as_number(pos.get("theta", 0.0), f"nodes[{i}].nodePosition.theta")
        nodes.append({"node_id": node_id, "sequence_id": seq, "released": released,
                      "x": x, "y": y, "theta": theta})

    raw_edges = raw.get("edges", [])
    _require(isinstance(raw_edges, list), "edges 必须为数组")
    _require(len(raw_edges) == len(nodes) - 1,
             f"edges 数量必须为 len(nodes)-1={len(nodes) - 1}, 实际: {len(raw_edges)}")
    edges = []
    for i, re_ in enumerate(raw_edges):
        _require(isinstance(re_, dict), f"edges[{i}] 必须为对象")
        edge_id = _as_str(re_.get("edgeId"), f"edges[{i}].edgeId")
        seq = _as_int(re_.get("sequenceId"), f"edges[{i}].sequenceId")
        start_id = _as_str(re_.get("startNodeId"), f"edges[{i}].startNodeId")
        end_id = _as_str(re_.get("endNodeId"), f"edges[{i}].endNodeId")
        released = _as_bool(re_.get("released"), f"edges[{i}].released")
        _require(start_id == nodes[i]["node_id"],
                 f"edges[{i}].startNodeId({start_id}) 必须等于 nodes[{i}].nodeId({nodes[i]['node_id']})")
        _require(end_id == nodes[i + 1]["node_id"],
                 f"edges[{i}].endNodeId({end_id}) 必须等于 nodes[{i + 1}].nodeId({nodes[i + 1]['node_id']})")
        traj = re_.get("trajectory")
        _require(isinstance(traj, dict), f"edges[{i}].trajectory 必须为对象")
        cps = traj.get("controlPoints")
        _require(isinstance(cps, list), f"edges[{i}].trajectory.controlPoints 必须为数组")
        polyline = []
        for j, cp in enumerate(cps):
            _require(isinstance(cp, dict), f"edges[{i}].trajectory.controlPoints[{j}] 必须为对象")
            polyline.append((_as_number(cp.get("x"), f"edges[{i}].cp[{j}].x"),
                             _as_number(cp.get("y"), f"edges[{i}].cp[{j}].y")))
        _require(len(polyline) >= 2,
                 f"edges[{i}].trajectory.controlPoints 至少需要 2 个控制点")
        edges.append({"edge_id": edge_id, "sequence_id": seq,
                      "start_node_id": start_id, "end_node_id": end_id,
                      "released": released, "polyline": polyline})
    return {"order_id": order_id, "order_update_id": update_id, "nodes": nodes, "edges": edges}


# =============================================================================
# 虚拟小车运动模型 (released 制动守卫 + 折线步进 + 断点恢复)
# =============================================================================
class VirtualVehicle(object):
    """沿订单折线行驶的虚拟小车。

    状态语义:
      - edge_index = k: 已确认 nodes[k] (lastNodeId), 正在/即将行驶 edges[k]
      - s: 沿 edges[k] 折线已行驶的弧长 (米)
      - finished: 已确认最终节点 (订单走完)
      - canceled: 收到 cancelOrder, 停止响应当前订单 (等待新订单)
    released=false 制动: edges[k] 未放行时 s 冻结, 车停在 nodes[k] 等待更新订单。
    """

    def __init__(self, serial, speed=1.0, battery=98.5):
        self.serial = serial
        self.speed = float(speed)
        self.battery = float(battery)
        self.header_id = 0
        self.order = None
        self.raw_order = None          # 原始订单 dict (落盘用)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_node_id = ""
        self.last_node_sequence_id = 0
        self.edge_index = 0
        self.s = 0.0
        self.finished = False
        self.canceled = False
        self._cum = []                 # 当前边折线累计弧长
        self._edge_len = 0.0

    # ---- 订单应用 -----------------------------------------------------------
    def apply_order(self, raw):
        """接受新订单 / 同单更新。返回动作描述字符串 ("new"/"update"/"ignored")。"""
        order = parse_order(raw)
        if self.order is not None and order["order_id"] == self.order["order_id"]:
            if order["order_update_id"] <= self.order["order_update_id"]:
                return "ignored"  # 重复/过期更新: 幂等忽略
            # 同单更新: 保留位置与进度, 仅替换订单 (released 变化可解除制动)
            self.order = order
            self.raw_order = raw
            self.canceled = False
            self._rebuild_geometry(clamp_progress=True)
            self.finished = self.edge_index >= len(self.order["edges"])
            return "update"
        # 全新订单: 传送至 node0 (VDA 5050 新单从首节点起步的调度语义)
        self.order = order
        self.raw_order = raw
        node0 = order["nodes"][0]
        self.x, self.y, self.yaw = node0["x"], node0["y"], node0["theta"]
        self.last_node_id = node0["node_id"]
        self.last_node_sequence_id = node0["sequence_id"]
        self.edge_index = 0
        self.s = 0.0
        self.canceled = False
        self.finished = len(order["edges"]) == 0
        self._rebuild_geometry()
        return "new"

    def _rebuild_geometry(self, clamp_progress=False):
        if self.order is None or self.edge_index >= len(self.order["edges"]):
            self._cum = []
            self._edge_len = 0.0
            return
        polyline = self.order["edges"][self.edge_index]["polyline"]
        cum = [0.0]
        for i in range(1, len(polyline)):
            dx = polyline[i][0] - polyline[i - 1][0]
            dy = polyline[i][1] - polyline[i - 1][1]
            cum.append(cum[-1] + math.hypot(dx, dy))
        self._cum = cum
        self._edge_len = cum[-1]
        if clamp_progress and self.s > self._edge_len:
            self.s = self._edge_len

    def tick(self, dt):
        """推进 dt 秒。返回 "moving"/"halted"/"arrived"/"idle"。"""
        if self.order is None or self.finished or self.canceled:
            return "idle"
        if self.edge_index >= len(self.order["edges"]):
            self.finished = True
            return "arrived"
        edge = self.order["edges"][self.edge_index]
        if not edge["released"]:
            return "halted"  # released=false 制动守卫: 停车等待更新订单
        self.s += self.speed * float(dt)
        if self.s >= self._edge_len and self._edge_len > 0.0:
            # 确认到达下一个节点
            self.edge_index += 1
            self.s = 0.0
            node = self.order["nodes"][self.edge_index]
            self.x, self.y, self.yaw = node["x"], node["y"], node["theta"]
            self.last_node_id = node["node_id"]
            self.last_node_sequence_id = node["sequence_id"]
            self._rebuild_geometry()
            if self.edge_index >= len(self.order["edges"]):
                self.finished = True
                return "arrived"
            return "moving"
        self._interpolate()
        return "moving"

    def _interpolate(self):
        polyline = self.order["edges"][self.edge_index]["polyline"]
        cum = self._cum
        # 二分定位 s 所在线段
        lo, hi = 0, len(cum) - 1
        while lo < hi:
            mid = (lo + hi) // 2
            if cum[mid] < self.s:
                lo = mid + 1
            else:
                hi = mid
        i = max(1, lo)
        seg = cum[i] - cum[i - 1]
        t = 0.0 if seg <= 0.0 else (self.s - cum[i - 1]) / seg
        x0, y0 = polyline[i - 1]
        x1, y1 = polyline[i]
        self.x = x0 + (x1 - x0) * t
        self.y = y0 + (y1 - y0) * t
        if seg > 1e-9:
            self.yaw = math.atan2(y1 - y0, x1 - x0)

    def cancel(self):
        """cancelOrder instantAction: 停止当前订单响应。"""
        if self.order is not None and not self.finished:
            self.canceled = True
            return True
        return False

    # ---- 报文构建 -----------------------------------------------------------
    def build_state(self, manufacturer):
        self.header_id += 1
        state = {
            "headerId": self.header_id,
            "timestamp": _iso_now(),
            "version": PROTOCOL_VERSION,
            "manufacturer": manufacturer,
            "serialNumber": self.serial,
            "lastNodeId": self.last_node_id,
            "lastNodeSequenceId": self.last_node_sequence_id,
            "agvPosition": {
                "x": round(self.x, 6),
                "y": round(self.y, 6),
                "theta": round(self.yaw, 6),
                "mapId": "map",
                "positionInitialized": True,
            },
            "velocity": {"vx": 0.0, "vy": 0.0, "omega": 0.0},
            "batteryState": {"batteryCharge": round(self.battery, 2), "charging": False},
            "safetyState": {"eStop": "none", "fieldViolation": False},
            "operatingMode": "AUTO",
            "errors": [],
        }
        if self.order is not None and not self.canceled:
            state["orderId"] = self.order["order_id"]
            state["orderUpdateId"] = self.order["order_update_id"]
        return json.dumps(state, ensure_ascii=False)

    # ---- 状态文件 (断链重启断点恢复) ---------------------------------------
    def save_state(self, path):
        data = {
            "header_id": self.header_id,
            "battery": self.battery,
            "x": self.x, "y": self.y, "yaw": self.yaw,
            "last_node_id": self.last_node_id,
            "last_node_sequence_id": self.last_node_sequence_id,
            "edge_index": self.edge_index,
            "s": self.s,
            "finished": self.finished,
            "canceled": self.canceled,
            "raw_order": self.raw_order,
            "ts": _iso_now(),
        }
        tmp = path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False)
        os.replace(tmp, path)  # 原子落盘

    def load_state(self, path):
        """从状态文件恢复。返回 True 表示有可恢复状态。"""
        if not path or not os.path.isfile(path):
            return False
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
        except (OSError, ValueError):
            return False
        self.header_id = int(data.get("header_id", 0))
        self.battery = float(data.get("battery", self.battery))
        self.x = float(data.get("x", 0.0))
        self.y = float(data.get("y", 0.0))
        self.yaw = float(data.get("yaw", 0.0))
        self.last_node_id = str(data.get("last_node_id", ""))
        self.last_node_sequence_id = int(data.get("last_node_sequence_id", 0))
        self.edge_index = int(data.get("edge_index", 0))
        self.s = float(data.get("s", 0.0))
        self.finished = bool(data.get("finished", False))
        self.canceled = bool(data.get("canceled", False))
        raw = data.get("raw_order")
        if raw:
            try:
                self.apply_order(raw)
                # apply_order 会重置进度, 恢复断点进度
                self.edge_index = int(data.get("edge_index", 0))
                self.s = float(data.get("s", 0.0))
                self.finished = bool(data.get("finished", False))
                self.canceled = bool(data.get("canceled", False))
                self._rebuild_geometry(clamp_progress=True)
                self.x = float(data.get("x", self.x))
                self.y = float(data.get("y", self.y))
                self.yaw = float(data.get("yaw", self.yaw))
                self.last_node_id = str(data.get("last_node_id", self.last_node_id))
                self.last_node_sequence_id = int(
                    data.get("last_node_sequence_id", self.last_node_sequence_id))
                return True
            except OrderError:
                return False
        return True


def _iso_now():
    """UTC ISO 8601 毫秒时间戳, 与 C++ CurrentIsoTimestamp 格式一致。"""
    now = time.time()
    millis = int((now - int(now)) * 1000)
    return time.strftime("%Y-%m-%dT%H:%M:%S", time.gmtime(now)) + ".%03dZ" % millis


# =============================================================================
# --self-test 自检模式 (无 MQTT / 无第三方依赖)
# =============================================================================
def _mk_order(order_id, update_id, nodes_spec, edges_spec):
    """便捷构造订单 dict。nodes_spec: [(id, seq, released, x, y), ...];
    edges_spec: [(id, seq, released, polyline), ...]"""
    nodes = []
    for nid, seq, rel, x, y in nodes_spec:
        nodes.append({"nodeId": nid, "sequenceId": seq, "released": rel,
                      "nodePosition": {"x": x, "y": y, "theta": 0.0, "mapId": "map"},
                      "actions": []})
    edges = []
    for eid, seq, rel, poly in edges_spec:
        edges.append({"edgeId": eid, "sequenceId": seq,
                      "startNodeId": "", "endNodeId": "", "released": rel,
                      "maxSpeed": 1.0,
                      "trajectory": {"degree": 1,
                                     "controlPoints": [{"x": px, "y": py, "theta": 0.0}
                                                       for px, py in poly]},
                      "actions": []})
    # 回填 start/end node id
    for i, e in enumerate(edges):
        e["startNodeId"] = nodes[i]["nodeId"]
        e["endNodeId"] = nodes[i + 1]["nodeId"]
    return {"headerId": 1, "timestamp": _iso_now(), "version": PROTOCOL_VERSION,
            "manufacturer": "RobotManufacturer", "serialNumber": "mock_selftest",
            "orderId": order_id, "orderUpdateId": update_id,
            "nodes": nodes, "edges": edges}


def self_test():
    """自检: 订单解析 / 折线步进 / released 制动等待 / 同单更新恢复 / 新单传送。
    返回 0 全部通过, 1 任一失败。"""
    failures = []

    def check(name, fn):
        try:
            fn()
            print("[SELF-TEST] PASS  %s" % name)
        except OrderError as e:
            failures.append(name)
            print("[SELF-TEST] FAIL  %s: %s" % (name, e))
        except AssertionError as e:
            failures.append(name)
            print("[SELF-TEST] FAIL  %s: %s" % (name, e))

    # --- T1: 合法订单解析 (sequence=i*2 契约) ---------------------------------
    def t1_parse_valid():
        raw = _mk_order("st_1", 0,
                        [("A", 0, True, 0.0, 0.0), ("B", 2, True, 3.0, 0.0),
                         ("C", 4, True, 3.0, 4.0)],
                        [("eAB", 1, True, [(0.0, 0.0), (1.5, 0.0), (3.0, 0.0)]),
                         ("eBC", 3, True, [(3.0, 0.0), (3.0, 2.0), (3.0, 4.0)])])
        o = parse_order(raw)
        assert o["order_id"] == "st_1" and o["order_update_id"] == 0
        assert [n["sequence_id"] for n in o["nodes"]] == [0, 2, 4], "node sequence 必须为 i*2"
        assert [e["sequence_id"] for e in o["edges"]] == [1, 3], "edge sequence 必须为 startNode+1"
        assert len(o["edges"][0]["polyline"]) == 3
        assert all(e["released"] for e in o["edges"])

    # --- T2: 非法订单拒绝 ----------------------------------------------------
    def t2_parse_invalid():
        good = _mk_order("st_bad", 0, [("A", 0, True, 0.0, 0.0), ("B", 2, True, 1.0, 0.0)],
                         [("e", 1, True, [(0.0, 0.0), (1.0, 0.0)])])
        for mutate, why in [
            (lambda o: o.pop("orderId"), "缺 orderId"),
            (lambda o: o.update(orderId=""), "空 orderId"),
            (lambda o: o.update(nodes=[]), "空 nodes"),
            (lambda o: o["nodes"][0].update(sequenceId=2), "sequenceId 重复"),
            (lambda o: o["nodes"][1].update(sequenceId=0), "sequenceId 逆序"),
            (lambda o: o["edges"][0]["trajectory"].update(controlPoints=[{"x": 0, "y": 0}]),
             "控制点不足"),
            (lambda o: o["edges"][0].update(edgeId=""), "空 edgeId"),
            (lambda o: o["nodes"][0]["nodePosition"].update(x="oops"), "非数字坐标"),
        ]:
            raw = json.loads(json.dumps(good))
            mutate(raw)
            try:
                parse_order(raw)
            except OrderError:
                continue
            raise AssertionError("应拒绝: %s" % why)

    # --- T3: 折线步进到终点 (位置沿路径、lastNode 推进、总长≈7m) --------------
    def t3_stepping():
        raw = _mk_order("st_move", 0,
                        [("A", 0, True, 0.0, 0.0), ("B", 2, True, 3.0, 0.0),
                         ("C", 4, True, 3.0, 4.0)],
                        [("eAB", 1, True, [(0.0, 0.0), (1.5, 0.0), (3.0, 0.0)]),
                         ("eBC", 3, True, [(3.0, 0.0), (3.0, 2.0), (3.0, 4.0)])])
        veh = VirtualVehicle("mock_selftest", speed=1.0)
        assert veh.apply_order(raw) == "new"
        assert veh.last_node_id == "A" and (veh.x, veh.y) == (0.0, 0.0)
        dist = 0.0
        prev = (veh.x, veh.y)
        on_path = True
        arrived_at = None
        for step in range(500):  # 50s 上限
            st = veh.tick(0.1)
            cur = (veh.x, veh.y)
            dist += math.hypot(cur[0] - prev[0], cur[1] - prev[1])
            prev = cur
            # 位置必须始终落在 L 形路径上
            on_seg1 = abs(cur[1]) < 1e-6 and -1e-6 <= cur[0] <= 3.0 + 1e-6
            on_seg2 = abs(cur[0] - 3.0) < 1e-6 and -1e-6 <= cur[1] <= 4.0 + 1e-6
            if not (on_seg1 or on_seg2):
                on_path = False
            if st == "arrived":
                arrived_at = step
                break
        assert arrived_at is not None, "未在时限内到达终点"
        assert on_path, "行驶轨迹偏离折线"
        assert 6.9 <= dist <= 7.2, "累计行程应≈7m, 实际 %.3f" % dist
        assert veh.last_node_id == "C" and veh.last_node_sequence_id == 4
        assert abs(veh.x - 3.0) < 1e-6 and abs(veh.y - 4.0) < 1e-6
        assert arrived_at * 0.1 >= 6.5, "到达过快(速度未生效?): %.1fs" % (arrived_at * 0.1)

    # --- T4: released=false 制动等待 + 同单更新恢复 --------------------------
    def t4_released_halt():
        nodes = [("A", 0, True, 0.0, 0.0), ("B", 2, True, 3.0, 0.0),
                 ("C", 4, True, 3.0, 4.0)]
        poly_ab = [(0.0, 0.0), (1.5, 0.0), (3.0, 0.0)]
        poly_bc = [(3.0, 0.0), (3.0, 2.0), (3.0, 4.0)]
        raw1 = _mk_order("st_halt", 0, nodes,
                         [("eAB", 1, True, poly_ab), ("eBC", 3, False, poly_bc)])
        veh = VirtualVehicle("mock_selftest", speed=1.0)
        veh.apply_order(raw1)
        for _ in range(200):
            if veh.tick(0.1) == "halted":
                break
        assert veh.last_node_id == "B", "应在 B 处被未放行边制动, 实际 lastNode=%s" % veh.last_node_id
        assert veh.edge_index == 1
        # 持续 tick 必须保持静止 (released=false 守卫)
        bx, by = veh.x, veh.y
        for _ in range(50):
            st = veh.tick(0.1)
            assert st == "halted"
            assert abs(veh.x - bx) < 1e-9 and abs(veh.y - by) < 1e-9, "released=false 时仍在移动"
        # 同单更新 (updateId+1, 全放行) 后恢复并走完
        raw2 = _mk_order("st_halt", 1, nodes,
                         [("eAB", 1, True, poly_ab), ("eBC", 3, True, poly_bc)])
        assert veh.apply_order(raw2) == "update"
        assert (veh.x, veh.y) == (bx, by), "更新订单不得改变车辆位置"
        got = None
        for _ in range(300):
            got = veh.tick(0.1)
            if got == "arrived":
                break
        assert got == "arrived", "更新放行后未恢复行驶到终点"
        assert veh.last_node_id == "C"
        # 过期更新 (updateId 相同/更小) 必须被幂等忽略
        raw3 = _mk_order("st_halt", 1, nodes,
                         [("eAB", 1, False, poly_ab), ("eBC", 3, False, poly_bc)])
        assert veh.apply_order(raw3) == "ignored"
        assert veh.order["edges"][1]["released"] is True, "过期订单不得覆盖当前订单"

    # --- T5: 新 orderId 传送至 node0 (多任务连续派发语义) --------------------
    def t5_teleport():
        raw1 = _mk_order("st_t1", 0, [("A", 0, True, 0.0, 0.0), ("B", 2, True, 3.0, 0.0)],
                         [("e", 1, True, [(0.0, 0.0), (3.0, 0.0)])])
        raw2 = _mk_order("st_t2", 0, [("X", 0, True, 10.0, 10.0), ("Y", 2, True, 11.0, 10.0)],
                         [("e", 1, True, [(10.0, 10.0), (11.0, 10.0)])])
        veh = VirtualVehicle("mock_selftest", speed=1.0)
        veh.apply_order(raw1)
        for _ in range(30):
            veh.tick(0.1)
        assert veh.x > 0.5, "应已驶离 A"
        assert veh.apply_order(raw2) == "new"
        assert abs(veh.x - 10.0) < 1e-9 and abs(veh.y - 10.0) < 1e-9, "新单应传送至 node0"
        assert veh.last_node_id == "X" and veh.last_node_sequence_id == 0

    # --- T6: state 报文契约 (DeserializeVda5050State 必需字段) ----------------
    def t6_state_contract():
        raw = _mk_order("st_state", 0, [("A", 0, True, 1.5, 2.0), ("B", 2, True, 8.0, 5.5)],
                        [("e", 1, True, [(1.5, 2.0), (8.0, 5.5)])])
        veh = VirtualVehicle("mock_amr_01", speed=1.0, battery=97.3)
        veh.apply_order(raw)
        veh.tick(0.5)
        state = json.loads(veh.build_state("RobotManufacturer"))
        assert state["serialNumber"] == "mock_amr_01"
        assert state["manufacturer"] == "RobotManufacturer"
        assert isinstance(state["headerId"], int) and state["headerId"] == 1
        for k in ("x", "y", "theta"):
            assert isinstance(state["agvPosition"][k], (int, float))
        assert state["orderId"] == "st_state"
        assert state["lastNodeId"] == "A" and state["lastNodeSequenceId"] == 0
        assert isinstance(state["batteryState"]["batteryCharge"], (int, float))
        assert state["safetyState"]["eStop"] == "none"
        # headerId 自增
        veh.build_state("RobotManufacturer")
        s2 = json.loads(veh.build_state("RobotManufacturer"))
        assert s2["headerId"] == 3, "headerId 应单调递增"
        # 取消后不再携带 orderId
        assert veh.cancel() is True
        s3 = json.loads(veh.build_state("RobotManufacturer"))
        assert "orderId" not in s3, "取消后 state 不应携带 orderId"

    # --- T7: 状态文件保存/恢复 (断链重启断点) --------------------------------
    def t7_state_file_roundtrip():
        import tempfile
        raw = _mk_order("st_persist", 0, [("A", 0, True, 0.0, 0.0), ("B", 2, True, 6.0, 0.0)],
                        [("e", 1, True, [(0.0, 0.0), (6.0, 0.0)])])
        veh = VirtualVehicle("mock_amr_01", speed=1.0)
        veh.apply_order(raw)
        for _ in range(10):
            veh.tick(0.1)
        assert veh.x > 0.5
        with tempfile.TemporaryDirectory() as td:
            path = os.path.join(td, "state.json")
            veh.save_state(path)
            veh2 = VirtualVehicle("mock_amr_01", speed=1.0)
            assert veh2.load_state(path) is True
            assert veh2.order["order_id"] == "st_persist"
            assert abs(veh2.x - veh.x) < 1e-9 and abs(veh2.y - veh.y) < 1e-9
            assert veh2.last_node_id == "A" and veh2.edge_index == 0
            # 恢复后继续行驶能走完
            got = None
            for _ in range(100):
                got = veh2.tick(0.1)
                if got == "arrived":
                    break
            assert got == "arrived" and veh2.last_node_id == "B"

    print("[SELF-TEST] ==== mock_vda5050_vehicle 自检开始 ====")
    check("T1 合法订单解析 (sequence 契约)", t1_parse_valid)
    check("T2 非法订单拒绝", t2_parse_invalid)
    check("T3 折线步进到终点", t3_stepping)
    check("T4 released=false 制动 + 同单更新恢复", t4_released_halt)
    check("T5 新单传送至 node0", t5_teleport)
    check("T6 state 报文契约", t6_state_contract)
    check("T7 状态文件断点恢复", t7_state_file_roundtrip)
    if failures:
        print("[SELF-TEST] ==== %d 项失败: %s ====" % (len(failures), ", ".join(failures)))
        return 1
    print("[SELF-TEST] ==== 全部通过 ====")
    return 0


# =============================================================================
# MQTT 运行时 (paho-mqtt 1.x 旧式 API)
# =============================================================================
def run_mqtt(args):
    try:
        import paho.mqtt.client as mqtt
    except ImportError:
        print("[ERROR] 需要 python3-paho-mqtt (apt install python3-paho-mqtt)", file=sys.stderr)
        return 2

    base = "uagv/v2/%s/%s" % (args.manufacturer, args.serial)
    topic_order = base + "/order"
    topic_actions = base + "/instantActions"
    topic_state = base + "/state"
    topic_conn = base + "/connection"

    veh = VirtualVehicle(args.serial, speed=args.speed, battery=args.battery)
    resumed = veh.load_state(args.state_file)
    print("[MOCK] serial=%s base=%s broker=%s:%d speed=%.2fm/s resumed=%s" %
          (args.serial, base, args.host, args.port, args.speed, resumed), flush=True)

    def conn_payload(state):
        return json.dumps({
            "headerId": 0, "timestamp": _iso_now(), "version": PROTOCOL_VERSION,
            "manufacturer": args.manufacturer, "serialNumber": args.serial,
            "connectionState": state,
        }, ensure_ascii=False)

    client = mqtt.Client(client_id="mock_vda5050_%s" % args.serial, clean_session=True)
    # LWT: kill -9 / 网络猝死时 broker 侧自动发布 CONNECTIONBROKEN (retained)
    client.will_set(topic_conn, payload=conn_payload("CONNECTIONBROKEN"), qos=1, retain=True)

    def on_connect(client, userdata, flags, rc):
        connected["ok"] = (rc == 0)
        if rc != 0:
            print("[MOCK][WARN] MQTT 连接失败 rc=%s, 稍后由 paho 重连" % rc, flush=True)
            return
        print("[MOCK] MQTT connected, 订阅 order/instantActions", flush=True)
        client.subscribe(topic_order, qos=1)
        client.subscribe(topic_actions, qos=1)
        client.publish(topic_conn, conn_payload("ONLINE"), qos=1, retain=True)

    def on_message(client, userdata, msg):
        try:
            raw = json.loads(msg.payload.decode("utf-8"))
        except ValueError as e:
            print("[MOCK][WARN] 非法 JSON (%s): %.128s" % (e, msg.payload[:128]), flush=True)
            return
        if msg.topic == topic_order:
            try:
                action = veh.apply_order(raw)
            except OrderError as e:
                print("[MOCK][WARN] 拒绝非法订单 orderId=%s: %s" %
                      (raw.get("orderId"), e), flush=True)
                return
            if action == "ignored":
                print("[MOCK] 忽略过期/重复订单 orderId=%s updateId=%s" %
                      (raw.get("orderId"), raw.get("orderUpdateId")), flush=True)
                return
            print("[MOCK] 接收订单 (%s): orderId=%s updateId=%s nodes=%d edges=%d "
                  "released=%s -> (%.3f, %.3f)" %
                  (action, veh.order["order_id"], veh.order["order_update_id"],
                   len(veh.order["nodes"]), len(veh.order["edges"]),
                   [e["released"] for e in veh.order["edges"]], veh.x, veh.y), flush=True)
            veh.save_state(args.state_file)
        elif msg.topic == topic_actions:
            actions = raw.get("instantActions", []) if isinstance(raw, dict) else []
            for act in actions:
                at = act.get("actionType", "") if isinstance(act, dict) else ""
                if at == "cancelOrder":
                    ok = veh.cancel()
                    print("[MOCK] instantAction cancelOrder: %s" %
                          ("已取消当前订单" if ok else "无进行中订单"), flush=True)
                    veh.save_state(args.state_file)
                else:
                    print("[MOCK][WARN] 未实现的 instantAction: %s" % at, flush=True)

    client.on_connect = on_connect
    client.on_message = on_message
    connected = {"ok": False}

    try:
        # keepalive=5: 断链注入(kill -9)后 broker 最快 7.5s 内判定死亡并触发 LWT,
        # 避免默认 60s keepalive 让 e2e 等待过久
        client.connect(args.host, args.port, keepalive=5)
    except OSError as e:
        print("[ERROR] 无法连接 broker %s:%d: %s" % (args.host, args.port, e), file=sys.stderr)
        return 2
    client.loop_start()

    start = time.time()
    last_state_pub = 0.0
    last_heartbeat = start
    last_save = start
    stopping = {"flag": False}

    def _sig_handler(signum, frame):
        stopping["flag"] = True

    signal.signal(signal.SIGTERM, _sig_handler)
    signal.signal(signal.SIGINT, _sig_handler)

    loop_dt = 0.02
    last_tick = time.time()
    try:
        while not stopping["flag"]:
            now = time.time()
            elapsed = now - start
            if args.run_seconds > 0 and elapsed >= args.run_seconds:
                print("[MOCK] --run-seconds=%d 到时, 正常退出" % args.run_seconds, flush=True)
                break
            if args.drop_after_n_seconds > 0 and elapsed >= args.drop_after_n_seconds:
                # 断链注入: 先落盘, 再 kill -9 自身 —— broker 检测 TCP 猝死触发 LWT
                veh.save_state(args.state_file)
                print("[MOCK] --drop-after-n-seconds=%d 到时, SIGKILL 自身注入断链 (LWT)" %
                      args.drop_after_n_seconds, flush=True)
                sys.stdout.flush()
                os.kill(os.getpid(), signal.SIGKILL)

            # 用真实流逝时间驱动 tick, 保证仿真速度 == 配置速度(实时)
            dt = min(now - last_tick, 0.1)
            last_tick = now
            veh.tick(dt)
            if args.battery_drain > 0.0:
                veh.battery = max(5.0, veh.battery - args.battery_drain * dt)

            if connected["ok"] and now - last_heartbeat >= args.heartbeat_seconds:
                client.publish(topic_conn, conn_payload("ONLINE"), qos=1, retain=True)
                last_heartbeat = now

            if connected["ok"] and now - last_state_pub >= 1.0 / args.state_hz:
                client.publish(topic_state, veh.build_state(args.manufacturer), qos=0)
                last_state_pub = now

            if args.state_file and now - last_save >= STATE_SAVE_PERIOD_SEC:
                veh.save_state(args.state_file)
                last_save = now

            time.sleep(loop_dt)
    finally:
        if args.state_file:
            try:
                veh.save_state(args.state_file)
            except OSError:
                pass
        try:
            client.loop_stop()
            client.disconnect()
        except Exception:
            pass
        print("[MOCK] 已退出", flush=True)
    return 0


# =============================================================================
# CLI
# =============================================================================
def main():
    p = argparse.ArgumentParser(description="VDA 5050 虚拟小车模拟器 (Task 6)")
    p.add_argument("--host", default="127.0.0.1", help="MQTT broker 地址 (默认 127.0.0.1)")
    p.add_argument("--port", type=int, default=1883, help="MQTT broker 端口 (默认 1883)")
    p.add_argument("--manufacturer", default="RobotManufacturer", help="manufacturer 段")
    p.add_argument("--serial", default="mock_amr_01", help="serialNumber 段 (车辆 id)")
    p.add_argument("--speed", type=float, default=1.0, help="行驶速度 m/s (默认 1.0)")
    p.add_argument("--battery", type=float, default=98.5, help="初始电量百分比")
    p.add_argument("--battery-drain", type=float, default=0.001,
                   help="每秒电量衰减 (默认 0.001%%/s, 0 = 不衰减)")
    p.add_argument("--state-hz", type=float, default=DEFAULT_STATE_HZ,
                   help="state 上报频率 Hz (默认 2.0)")
    p.add_argument("--heartbeat-seconds", type=float, default=DEFAULT_HEARTBEAT_SEC,
                   help="connection ONLINE retained 心跳周期秒 (默认 5.0)")
    p.add_argument("--run-seconds", type=float, default=0.0,
                   help="运行 N 秒后正常退出 (0 = 持续运行)")
    p.add_argument("--drop-after-n-seconds", type=float, default=0.0,
                   help="运行 N 秒后 kill -9 自身注入断链 (0 = 关闭)")
    p.add_argument("--state-file", default="",
                   help="状态持久化文件路径 (断链重启断点恢复; 空 = 不落盘)")
    p.add_argument("--self-test", action="store_true",
                   help="不连 MQTT 的自检模式 (订单解析/步进/制动), 0=通过 1=失败")
    args = p.parse_args()

    if args.self_test:
        sys.exit(self_test())
    sys.exit(run_mqtt(args))


if __name__ == "__main__":
    main()
