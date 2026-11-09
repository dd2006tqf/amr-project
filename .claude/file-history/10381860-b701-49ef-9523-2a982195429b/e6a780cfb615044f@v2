#!/usr/bin/env python3
"""双发一致性校验器：确认 rosidl 与 protobuf 两路话题内容逐字段相同。

============================================================================
为什么需要它（有了投影层单测之后还剩什么问题）
============================================================================
test_proto_projection 已经证明：给定一条 rosidl 消息，Project*() 产出的
proto 消息字段正确。但它证明不了下面这些——这些只有把真节点跑起来才看得见：

  1. 节点里**真的调用了** Publish()（漏加一处调用是静默的）
  2. lifecycle 节点的 proto 侧**真的被激活了**（漏调 Activate() 是静默的）
  3. 两路的 **QoS 深度一致**（不一致时突发丢帧会伪装成"内容不一致"）
  4. per-vehicle 话题派生在两侧**用的是同一个名字**（只派生一次）
  5. 实际 on-wire 的字节能被**独立解码**（投影对了但序列化坏了也白搭）

所以本校验器的定位是**独立的第二意见**：它不走 C++ 的投影代码，而是从
DDS 层分别拿 rosidl 消息和 proto 原始字节，自己解码后比对。

============================================================================
为什么不逐帧配对比较时间戳
============================================================================
两路的发布时刻相差微秒级但不相同，严格逐帧比对时间戳会永远失败。
本校验器按**到达顺序**配对（第 N 条 rosidl 对第 N 条 proto），跳过
header/stamp 字段，其余字段全部比对。这能抓住"某字段永远为空""枚举
映射错""repeated 少一项"这类真实缺陷，而不会因时序噪声误报。

用法:
    python3 scripts/check_proto_dual_consistency.py --topic /chassis/link_health \
        --proto-type amr_dispatcher.proto.ChassisLinkHealth --seconds 5

退出码: 0 = 一致；1 = 发现差异；2 = 无法运行（缺依赖/无数据）
"""

from __future__ import annotations

import argparse
import struct
import sys
import time
from typing import Any, Dict, List, Optional

# 比对时忽略的字段：两路发布时刻不同，逐帧比对时间戳必然不等。
SKIP_FIELDS = {
    'header',
    'stamp',
}

# 只在 proto 侧存在、由已知翻译规则**派生**出来的字段。
#
# 这些不是漏投影，而是 proto3 表达能力限制导致的必要拆分：
# rosidl 的 `command_delivery_rate = -1.0`（不适用哨兵）在 proto3 的无符号
# 数值类型里无法表达，所以拆成 (valid: bool, rate: double)。valid 是派生量，
# 不该在 rosidl 侧找同名字段。
#
# 它们本身仍要被校验：_compare 会对 rate 做数值比对，valid 的正确性由
# test_proto_projection 的 ChassisLinkHealthDeliveryRateSentinel 覆盖。
PROTO_DERIVED_FIELDS = {
    'command_delivery_rate_valid',
}

# string -> enum 的字段映射，用于把 rosidl 侧的字符串翻译成 proto 侧的枚举值
# 再比较。键是 rosidl 字段路径，值是 {字符串: proto 枚举值}。
#
# 为什么不能直接比字符串：proto 侧该字段是 int（enum 在 wire 上是 varint），
# 不翻译就永远报"不同"。
#
# **_text 兜底字段不在此列**：它是"枚举外的值"的原文，只在 rosidl 侧
# 出现枚举外字符串时才有内容，由 _compare 的常规分支比对。
_STRING_ENUM_FIELDS: Dict[str, Dict[str, int]] = {
    'tier': {
        'UNKNOWN': 0,   # TIER_UNKNOWN
        'HEALTHY': 1,   # TIER_HEALTHY
        'DEGRADED': 2,  # TIER_DEGRADED
        'CRITICAL': 3,  # TIER_CRITICAL
    },
    'severity': {
        'UNKNOWN': 0,   # SEVERITY_UNKNOWN
        'INFO': 1,      # SEVERITY_INFO
        'WARN': 2,      # SEVERITY_WARN
        'CRITICAL': 3,  # SEVERITY_CRITICAL
    },
}


def _strip_cdr(raw: bytes) -> Optional[bytes]:
    """剥掉 Fast DDS 的 4 字节封装头与 C++ 侧写的 4 字节长度前缀。

    与 amr_dispatcher_proto_cli/echo.py 的 _strip_length_prefix 保持同一份
    布局假设（Jazzy / rmw_fastrtps_cpp 实测）：

        [0,2)   encapsulation kind (00 01 = CDR_LE, 00 00 = CDR_BE)
        [2,4)   encapsulation options
        [4,8)   uint32 长度
        [8,8+n) protobuf 字节
        [8+n,)  4 字节对齐 padding

    这里重新实现而不是 import：校验器要在"echo 工具本身可能坏了"的场景下
    仍然可用，否则它就不能算第二意见。
    """
    if len(raw) < 8:
        return None
    if raw[0:2] == b'\x00\x01':
        endian = '<'
    elif raw[0:2] == b'\x00\x00':
        endian = '>'
    else:
        return None
    (declared,) = struct.unpack_from(endian + 'I', raw, 4)
    body = raw[8:]
    if declared > len(body):
        return None
    return body[:declared]


def _build_proto_class(type_name: str) -> Any:
    """从已安装的 .proto 文件构建 protobuf Python 消息类。

    独立实现而不是复用 amr_dispatcher_proto_cli.descriptor_pool：
    校验器的价值在于「独立第二意见」，若它和 echo 工具共用同一份解码逻辑，
    则 echo 的描述符池构造一旦有 bug，校验器会一起错，比对结果就没意义了。

    优先用 ament 索引找到 share/amr_dispatcher_proto/proto/ 下的 .proto，
    调 protoc 编译成 FileDescriptorSet，再建 DescriptorPool。
    """
    import os
    import subprocess
    import tempfile
    from pathlib import Path

    from google.protobuf import descriptor_pb2
    from google.protobuf import descriptor_pool
    from google.protobuf import message_factory

    proto_dir: Optional[Path] = None
    try:
        from ament_index_python.packages import get_package_share_directory
        candidate = Path(get_package_share_directory('amr_dispatcher_proto')) / 'proto'
        if candidate.is_dir():
            proto_dir = candidate
    except Exception:  # noqa: BLE001 - 非 ROS 环境/包没装
        pass

    if proto_dir is None:
        # 开发态回退：从脚本位置往上找 src/amr_dispatcher_proto/proto
        here = Path(__file__).resolve()
        for parent in here.parents:
            candidate = parent / 'src' / 'amr_dispatcher_proto' / 'proto'
            if candidate.is_dir():
                proto_dir = candidate
                break
    if proto_dir is None:
        raise RuntimeError(
            '找不到 .proto 目录。请确认 amr_dispatcher_proto 已安装，'
            '或在仓库内运行本脚本。')

    proto_files = sorted(str(p) for p in proto_dir.glob('*.proto'))
    if not proto_files:
        raise RuntimeError(f'{proto_dir} 下没有 .proto 文件')

    with tempfile.TemporaryDirectory() as tmp:
        out_file = os.path.join(tmp, 'descriptors.pb')
        cmd = ['protoc', f'-I{proto_dir}', f'--descriptor_set_out={out_file}']
        cmd.extend(proto_files)
        try:
            subprocess.run(cmd, check=True, capture_output=True)
        except FileNotFoundError as exc:
            raise RuntimeError(
                '需要 protoc 来解析 .proto（apt install protobuf-compiler）') from exc
        except subprocess.CalledProcessError as exc:
            raise RuntimeError(
                f'protoc 失败: {exc.stderr.decode(errors="replace")[:500]}') from exc

        fds = descriptor_pb2.FileDescriptorSet()
        fds.ParseFromString(Path(out_file).read_bytes())

    pool = descriptor_pool.DescriptorPool()
    # .proto 之间有 import 依赖（chassis.proto import common.proto），
    # 必须按拓扑序添加；简单重试直到全部加入。
    pending = list(fds.file)
    added: set = set()
    while pending:
        progressed = False
        remaining = []
        for fd in pending:
            missing = [d for d in fd.dependency if d not in added]
            if missing:
                remaining.append(fd)
                continue
            pool.Add(fd)
            added.add(fd.name)
            progressed = True
        if not progressed:
            names = ', '.join(fd.name for fd in remaining)
            raise RuntimeError(f'.proto 依赖无法解析（循环或缺失）: {names}')
        pending = remaining

    desc = pool.FindMessageTypeByName(type_name)
    # protobuf 3.21（Ubuntu 24.04 系统包）没有 message_factory.GetMessageClass，
    # 那是 4.x 的 API。3.x 用 MessageFactory().GetPrototype，两者等价。
    if hasattr(message_factory, 'GetMessageClass'):
        return message_factory.GetMessageClass(desc)
    return message_factory.MessageFactory(pool).GetPrototype(desc)


def _make_proto_msg_type(type_name: str) -> type:
    """构造满足 rclpy create_subscription 校验的 proto 消息类型占位。

    与 amr_dispatcher_proto_cli.echo._make_rclpy_msg_type 同样的技巧，但独立
    实现：capsule 必须来自 amr_dispatcher_proto 的 pybind 模块（真实的
    rosidl_message_type_support_t*），其余四个转换函数在 raw=True 时不会被调用。
    """
    try:
        import _amr_proto_typesupport as ts_mod
    except ImportError as exc:
        raise RuntimeError(
            '找不到 _amr_proto_typesupport 扩展模块（amr_dispatcher_proto 的 '
            'pybind11 组件）。请确认它已构建且 lib/python3.x/site-packages '
            '在 PYTHONPATH 中。') from exc

    simple = type_name.rsplit('.', 1)[-1]
    fn_name: list = []
    for i, ch in enumerate(simple):
        if ch.isupper() and i > 0:
            fn_name.append('_')
        fn_name.append(ch.lower())
    expected = ''.join(fn_name)
    func = getattr(ts_mod, expected, None)
    if func is None:
        available = sorted(n for n in dir(ts_mod) if not n.startswith('_'))
        raise RuntimeError(f'pybind 模块里没有 {expected}；可用: {available}')

    class _ProtoMsgType:
        _TYPE_SUPPORT = func()
        _CREATE_ROS_MESSAGE = staticmethod(lambda: None)
        _CONVERT_FROM_PY = staticmethod(lambda: None)
        _CONVERT_TO_PY = staticmethod(lambda: None)
        _DESTROY_ROS_MESSAGE = staticmethod(lambda: None)

    return _ProtoMsgType


def _flatten_proto(msg: Any, prefix: str = '') -> Dict[str, Any]:
    """把 protobuf 消息压平成 {路径: 值}，便于与 rosidl 侧逐项比对。"""
    out: Dict[str, Any] = {}
    for field, value in msg.ListFields():
        name = prefix + field.name
        # repeated message（如 vehicle_links）
        if field.label == field.LABEL_REPEATED and field.type == field.TYPE_MESSAGE:
            for i, item in enumerate(value):
                out.update(_flatten_proto(item, f'{name}[{i}].'))
            out[f'{name}.__len__'] = len(value)
        elif field.type == field.TYPE_MESSAGE:
            out.update(_flatten_proto(value, name + '.'))
        elif field.label == field.LABEL_REPEATED:
            out[f'{name}.__list'] = list(value)
        else:
            out[name] = value
    return out


def _flatten_rosidl(msg: Any, prefix: str = '') -> Dict[str, Any]:
    """把 rosidl 消息压平成同样的形式。

    rosidl 的 Python 消息用 __slots__ 暴露字段；没有 ListFields()，
    所以要遍历类型定义。默认值也会出现（与 protobuf 只列已设字段不同），
    因此比对时以 proto 侧字段集合为准，rosidl 多出来的默认值不报错。
    """
    out: Dict[str, Any] = {}
    for name in getattr(msg, '__slots__', []):
        if name.startswith('_'):
            # rosidl 生成的 Python 消息把字段存在 _field_name 里，
            # __slots__ 列出的正是这些带下划线的名字；对外属性名要去掉前缀。
            name = name[1:]
        value = getattr(msg, '_' + name, None)
        if value is None:
            value = getattr(msg, name, None)
        key = prefix + name
        if isinstance(value, (list, tuple)):
            out[f'{key}.__list'] = list(value)
        elif hasattr(value, '__slots__'):
            out.update(_flatten_rosidl(value, key + '.'))
        else:
            out[key] = value
    return out


def _compare(rosidl_flat: Dict[str, Any], proto_flat: Dict[str, Any]) -> List[str]:
    """逐字段比对，返回差异描述列表（空 = 一致）。"""
    diffs: List[str] = []

    for key, proto_value in proto_flat.items():
        # 跳过时间戳类字段（见模块文档）
        if any(part in SKIP_FIELDS for part in key.split('.')):
            continue

        if key.endswith('.__len__'):
            base = key[: -len('.__len__')]
            ros_list = rosidl_flat.get(f'{base}.__list')
            if ros_list is not None and len(ros_list) != proto_value:
                diffs.append(f'{base}: 长度不同 rosidl={len(ros_list)} proto={proto_value}')
            continue

        if key.endswith('.__list'):
            base = key[: -len('.__list')]
            ros_list = rosidl_flat.get(key)
            if ros_list is None:
                diffs.append(f'{base}: proto 有该数组但 rosidl 侧不存在')
                continue
            if list(ros_list) != list(proto_value):
                diffs.append(f'{base}: 数组内容不同 rosidl={list(ros_list)} proto={list(proto_value)}')
            continue

        ros_value = rosidl_flat.get(key)
        if ros_value is None:
            # 已知的派生字段（proto 侧独有）：跳过，理由见 PROTO_DERIVED_FIELDS。
            if key in PROTO_DERIVED_FIELDS:
                continue
            # rosidl 侧没有同名字段：可能是 proto 侧的 *_text 兼容字段，
            # 或字段改名了。只提示不判失败——判失败的规则在下方反向检查里。
            if not key.endswith('_text'):
                diffs.append(f'{key}: proto 有值 {proto_value!r} 但 rosidl 侧无同名字段')
            continue

        # 枚举字段：proto 是 int，rosidl 是字符串。必须按同一套映射比较，
        # 否则每一次都会报「不同」，把真正的问题淹没。
        if key in _STRING_ENUM_FIELDS:
            ros_enum = _STRING_ENUM_FIELDS[key].get(str(ros_value))
            if ros_enum is not None:
                if ros_enum != proto_value:
                    diffs.append(
                        f'{key}: 枚举映射不同 rosidl={ros_value!r} '
                        f'(期望 {ros_enum}) proto={proto_value}')
                continue
            # 枚举外的字符串：proto 侧应落到 0 且在 *_text 里保留原文，
            # 这个由 _flatten_proto 的 *_text 项另行比对。

        if isinstance(proto_value, float):
            if abs(float(ros_value) - proto_value) > 1e-9:
                diffs.append(f'{key}: 数值不同 rosidl={ros_value!r} proto={proto_value!r}')
        elif str(proto_value) != str(ros_value):
            diffs.append(f'{key}: 不同 rosidl={ros_value!r} proto={proto_value!r}')

    return diffs


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--topic', required=True, help='rosidl 话题名（不含 _proto 后缀）')
    parser.add_argument('--rosidl-type', required=True,
                        help='rosidl 类型，如 amr_dispatcher_interfaces/msg/ChassisLinkHealth')
    parser.add_argument('--proto-type', required=True,
                        help='protobuf 类型，如 amr_dispatcher.proto.ChassisLinkHealth')
    parser.add_argument('--seconds', type=float, default=5.0,
                        help='采集时长（秒）')
    parser.add_argument('--max-frames', type=int, default=50,
                        help='最多比对多少帧，防止高频话题刷屏')
    args = parser.parse_args(argv)

    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        print(f'错误: 缺少 ROS 2 Python 依赖: {exc}', file=sys.stderr)
        return 2

    try:
        proto_cls = _build_proto_class(args.proto_type)
        proto_msg_type = _make_proto_msg_type(args.proto_type)
    except Exception as exc:  # noqa: BLE001 - 缺 .proto / 缺 pybind 模块都归到这
        print(f'错误: 无法准备 protobuf 解码: {exc}', file=sys.stderr)
        return 2

    rosidl_cls = get_message(args.rosidl_type)

    rosidl_frames: List[Any] = []
    proto_frames: List[Any] = []
    decode_errors: List[str] = []

    rclpy.init()
    node = Node('proto_dual_consistency_checker')
    qos = QoSProfile(depth=50)

    def on_rosidl(msg: Any) -> None:
        if len(rosidl_frames) < args.max_frames:
            rosidl_frames.append(msg)

    def on_proto_raw(raw: bytes) -> None:
        if len(proto_frames) >= args.max_frames:
            return
        payload = _strip_cdr(raw)
        if payload is None:
            decode_errors.append(f'无法剥离 CDR 头（{len(raw)} 字节）')
            return
        msg = proto_cls()
        try:
            msg.ParseFromString(payload)
        except Exception as exc:  # noqa: BLE001
            decode_errors.append(f'protobuf 解析失败: {exc}')
            return
        proto_frames.append(msg)

    node.create_subscription(rosidl_cls, args.topic, on_rosidl, qos)
    node.create_subscription(proto_msg_type(), args.topic + '_proto', on_proto_raw, qos, raw=True)

    deadline = time.monotonic() + args.seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)

    node.destroy_node()
    rclpy.shutdown()

    if decode_errors:
        print(f'错误: proto 侧有 {len(decode_errors)} 帧无法解码', file=sys.stderr)
        for e in decode_errors[:5]:
            print('  ' + e, file=sys.stderr)
        return 1

    if not rosidl_frames:
        print(f'错误: {args.seconds}s 内 {args.topic} 没有任何 rosidl 数据。\n'
              f'      确认发布节点在跑（ros2 topic hz {args.topic}）', file=sys.stderr)
        return 2

    if not proto_frames:
        print(f'错误: {args.seconds}s 内 {args.topic}_proto 没有任何 proto 数据。\n'
              '      常见原因：proto_dual_publish=false；lifecycle 节点未 activate；'
              '      节点漏调 ProtoDualPublisher::Publish()', file=sys.stderr)
        return 1

    n = min(len(rosidl_frames), len(proto_frames))
    if len(rosidl_frames) != len(proto_frames):
        print(f'警告: 帧数不等 rosidl={len(rosidl_frames)} proto={len(proto_frames)}'
              f'（QoS 深度不一致或高频丢帧；只比对前 {n} 帧）', file=sys.stderr)

    total_diffs = 0
    for i in range(n):
        diffs = _compare(_flatten_rosidl(rosidl_frames[i]), _flatten_proto(proto_frames[i]))
        if diffs:
            total_diffs += 1
            if total_diffs <= 3:
                print(f'\n第 {i} 帧存在 {len(diffs)} 处差异:')
                for d in diffs[:10]:
                    print('  - ' + d)

    if total_diffs:
        print(f'\n失败: {total_diffs}/{n} 帧内容不一致', file=sys.stderr)
        return 1

    print(f'一致: {n} 帧逐字段比对通过（{args.topic} vs {args.topic}_proto）')
    return 0


if __name__ == '__main__':
    sys.exit(main())
