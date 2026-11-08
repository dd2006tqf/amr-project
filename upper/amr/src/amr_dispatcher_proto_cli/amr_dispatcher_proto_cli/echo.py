"""`ros2 amr echo` —— 解析 protobuf 编码的 ROS 话题。

为什么需要这个命令
------------------
`ros2 topic echo` 无法解析 protobuf 话题，原因是它的实现路径：

    echo.py
      → ros2topic.api.get_msg_class()
        → rosidl_runtime_py.utilities.get_message(type_name)
          → import_message_from_namespaced_type(...)   # 按类型名 import Python 模块
            → is_message(interface) 判据是 hasattr(interface, 'SLOT_TYPES')

protobuf 类型没有 rosidl 的 Python 生成包，因此 import 必然失败。
这是**架构性**的，不是配置问题，写插件也绕不过去。

本命令的做法是绕开整条 Python 消息类链路：
    1. 用 rclpy 的 raw 订阅（`raw=True`）直接拿原始 DDS 载荷 bytes
    2. 用 protobuf 的 Python 描述符池动态构建消息类
    3. 用 ParseFromString 解析

字节布局（rmw_fastrtps_cpp / Jazzy 实测，见 _strip_length_prefix）：

    [0,2)    encapsulation kind  00 01 = CDR_LE, 00 00 = CDR_BE
    [2,4)    encapsulation 选项
    [4,8)    uint32 长度（与 encapsulation 同字节序）
    [8,8+n)  protobuf 序列化字节
    [8+n,..) 4 字节对齐 padding（0~3 字节）

前 4 字节是 **fastrtps 加的**，不属于本方案；长度前缀那 4 字节才是
C++ 侧 cdr_serialize 写的。早期版本只跳过 4 字节，于是把封装头
（00 01 00 00 → 256）当成了长度，误报"载荷过短"。
"""

from __future__ import annotations

import struct
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy

from .descriptor_pool import ProtoSchemaError
from .descriptor_pool import registry


def _make_rclpy_msg_type(type_name: str) -> type:
    """构造一个满足 rclpy 检查的「消息类型」。

    为什么需要这个
    --------------
    rclpy.node.create_subscription 在**判断 raw 之前**就调用
    check_is_valid_msg_type()，它要求类型带有：
        _TYPE_SUPPORT            → 必须是 PyCapsule（指向 typesupport handle）
        _CREATE_ROS_MESSAGE      → 非 None
        _CONVERT_FROM_PY         → 非 None
        _CONVERT_TO_PY           → 非 None
        _DESTROY_ROS_MESSAGE     → 非 None

    正常 ROS 消息由 rosidl_generator_py 生成这些属性；protobuf 类型没有
    那套生成器，所以由 amr_dispatcher_proto 的 pybind11 模块提供 capsule
    （见 src/py_typesupport_module.cpp），其余四个在 raw 模式下不会被
    调用，只需非 None 以满足校验。

    诚实性说明：这不是伪造 typesupport。capsule 里装的是**真实的**
    rosidl_message_type_support_t*，由 C++ 侧的 GetProtobufTypeSupport
    构造，与 C++ 发布端用的是同一个 handle。伪的只是 Python 侧的四个
    转换函数占位符，而它们只在非 raw 模式下才被调用。
    """
    try:
        import _amr_proto_typesupport as ts_mod
    except ImportError as exc:
        raise ProtoSchemaError(
            '找不到 _amr_proto_typesupport 扩展模块。\n'
            '它是 amr_dispatcher_proto 包的 pybind11 组件，负责把 typesupport '
            'handle 以 PyCapsule 形式暴露给 rclpy。\n'
            '请确认 amr_dispatcher_proto 已完整构建（需要 pybind11-dev），'
            '且其 lib 目录在 PYTHONPATH 中。'
        ) from exc

    # 生成器产出的函数名是 proto 消息名的 snake_case，不带 type_support_ 前缀。
    fn_name = _proto_type_to_func_name(type_name)
    ts_fn = getattr(ts_mod, fn_name, None)
    if ts_fn is None:
        available = sorted(
            n for n in dir(ts_mod) if not n.startswith('_')
        )
        raise ProtoSchemaError(
            f'pybind11 模块里没有 {fn_name}（对应类型 {type_name}）。\n'
            f'可用函数: {available}'
        )

    class _ProtoMsgType:
        """raw 订阅用的类型占位。实例化后传给 create_subscription。"""

        _TYPE_SUPPORT = ts_fn()

        # 仅用于通过 check_is_valid_msg_type 的非 None 校验。
        # raw=True 时不会被调用（executors.py 走 take_message(msg_type, raw)）。
        _CREATE_ROS_MESSAGE = staticmethod(lambda: None)
        _CONVERT_FROM_PY = staticmethod(lambda: None)
        _CONVERT_TO_PY = staticmethod(lambda: None)
        _DESTROY_ROS_MESSAGE = staticmethod(lambda: None)

    return _ProtoMsgType


def _proto_type_to_func_name(type_name: str) -> str:
    """'amr_dispatcher.proto.ChassisLinkHealth' -> 'chassis_link_health'。

    与 scripts/gen_typesupport_bindings.py 的 snake() 保持一致。
    """
    simple = type_name.rsplit('.', 1)[-1]
    out: list[str] = []
    for i, ch in enumerate(simple):
        if ch.isupper() and i > 0:
            out.append('_')
        out.append(ch.lower())
    return ''.join(out)


class ProtoEchoNode(Node):
    """订阅 protobuf 话题并按收到顺序打印。"""

    def __init__(self, topic: str, type_name: str, count: int, qos: QoSProfile) -> None:
        super().__init__('proto_echo')
        self._count_limit = count
        self._received = 0
        # 收到的原始帧数（不论解码是否成功）。用于区分\"没有数据\"和
        # \"有数据但类型对不上\"——这两者的排障方向完全相反。
        self._transport_frames = 0
        self._msg_cls = registry.get_class(type_name)
        self._type_name = type_name

        # 构造满足 rclpy 校验的类型（真实 capsule + raw 模式占位属性）。
        msg_type = _make_rclpy_msg_type(type_name)

        # raw=True 是这里的核心：rclpy 不尝试构造 Python 消息对象，
        # 直接把 DDS 载荷以 bytes 形式交给回调。
        self._sub = self.create_subscription(
            msg_type(),
            topic,
            self._on_raw,
            qos,
            raw=True,
        )

    def _on_raw(self, raw: bytes) -> None:
        self._transport_frames += 1

        payload = self._strip_length_prefix(raw)
        if payload is None:
            self.get_logger().error(
                f'无法解析载荷（{len(raw)} 字节）：CDR 封装头或长度前缀不符合预期。'
                f'前 8 字节为 {bytes(raw[:8]).hex()}。'
                '若这确实来自 amr_dispatcher_proto 的发布端，请核对 '
                'echo.py 中 _strip_length_prefix 的布局假设。'
            )
            return

        msg = self._msg_cls()
        try:
            msg.ParseFromString(payload)
        except Exception as exc:  # noqa: BLE001 - protobuf 解析错误类型较多
            self.get_logger().error(
                f'protobuf 解析失败（载荷 {len(payload)} 字节）: {exc}\n'
                f'前 16 字节: {payload[:16].hex()}\n'
                '常见原因：--type 指定的 protobuf 类型与发布端不一致。'
            )
            return

        # 只有**成功解码**才计入 --count。否则类型不匹配时会一直刷错误日志、
        # 永远达不到上限（实测 249 帧全失败也不退出，只能靠外层 timeout）。
        self._received += 1

        # 用 protobuf 自带的文本格式输出，字段名与 .proto 完全一致，
        # 比 YAML 更贴近开发者对 protobuf 的预期。
        from google.protobuf import text_format
        print(f'--- [{self._received}] {self._type_name} ---')
        print(text_format.MessageToString(msg), end='')
        print()

        if self._count_limit and self._received >= self._count_limit:
            self.get_logger().info(
                f'已成功解析 {self._received} 条'
                f'（共收到 {self._transport_frames} 帧），达到上限，退出'
            )
            rclpy.shutdown()

    @staticmethod
    def _strip_length_prefix(raw: bytes) -> Optional[bytes]:
        """剥掉 CDR 封装头与长度前缀，返回 protobuf 字节。

        完整布局（Jazzy / rmw_fastrtps_cpp 实测）：

            [0,2)      encapsulation kind
                         00 01 -> CDR_LE（小端）
                         00 00 -> CDR_BE（大端）
            [2,4)      encapsulation options（当前恒为 00 00）
            [4,8)      uint32 长度，字节序由 encapsulation 决定
            [8,8+n)    protobuf 字节
            [8+n, ...) padding，补齐到 4 字节边界

        对应 C++ 侧（protobuf_typesupport.hpp 的 cdr_serialize）：
            cdr << n;                                    // [4,8) 这 4 字节
            cdr.serialize_array(..., n);                 // [8,8+n)

        为什么不能简单 \"raw[4:]\"：
        Fast DDS 在最外层多加了 4 字节封装头，它不是本方案写的。
        早期版本只跳 4 字节，读到的是封装头 00 01 00 00（小端=256），
        再和 52-4=48 比大小，于是每帧都误报\"载荷过短\"。
        这个错误在 249 帧连续测试中稳定复现，不是偶发。
        """
        if len(raw) < 8:
            return None

        encap = raw[0:4]
        if encap[0:2] == b'\x00\x01':
            endian = '<'
        elif encap[0:2] == b'\x00\x00':
            endian = '>'
        else:
            # 不是已知的 CDR 封装。可能对端不是 rmw_fastrtps_cpp，
            # 或数据不是本方案发的——报错比猜一个偏移更安全。
            return None

        (declared,) = struct.unpack_from(endian + 'I', raw, 4)
        body = raw[8:]

        if declared > len(body):
            # 长度大于实际剩余，说明端点/封装头判断有误。
            return None
        return body[:declared]


def build_qos(reliability: str, durability: str, depth: int) -> QoSProfile:
    """按命令行参数构造 QoS。默认可靠+易失，与大多数话题匹配。"""
    rel = {
        'reliable': QoSReliabilityPolicy.RELIABLE,
        'best_effort': QoSReliabilityPolicy.BEST_EFFORT,
    }[reliability]
    dur = {
        'volatile': QoSDurabilityPolicy.VOLATILE,
        'transient_local': QoSDurabilityPolicy.TRANSIENT_LOCAL,
    }[durability]
    return QoSProfile(depth=depth, reliability=rel, durability=dur)


def echo_topic(topic: str, type_name: str, count: int,
               reliability: str = 'reliable',
               durability: str = 'volatile',
               depth: int = 10) -> int:
    """命令入口。返回进程退出码。"""
    try:
        # 提前构建描述符池：在起订阅之前失败，能给出更清晰的报错。
        registry.get_class(type_name)
        # 提前验证 pybind11 typesupport 模块可用（否则要到建订阅时才炸）。
        _make_rclpy_msg_type(type_name)
    except ProtoSchemaError as exc:
        print(f'错误: {exc}')
        return 2

    qos = build_qos(reliability, durability, depth)

    rclpy.init()
    node = ProtoEchoNode(topic, type_name, count, qos)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0
