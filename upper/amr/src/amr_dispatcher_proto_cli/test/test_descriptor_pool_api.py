"""descriptor_pool 的兼容性与正确性测试。

================================================================================
为什么这个测试必须存在
================================================================================
2026-11-07 在部署环境发现 `ros2 amr echo` 完全不可用，报

    AttributeError: module 'google.protobuf.message_factory'
    has no attribute 'GetMessageClass'

根因是 `GetMessageClass` 只在 protobuf 4.22+ 存在，而 Ubuntu 24.04（ROS 2
Jazzy 的基线）自带的是 4.21.x。这个包在事件发生前**一个测试都没有**，所以
一个只在部署环境显形的 API 差异，直接击穿了

    `ros2 amr echo` 是唯一能读懂 *_proto 话题的工具

这条承诺（见 PROTOBUF_MIGRATION_PLAN.md §5.2）。话题在跑、数据在对，但运维
侧没有任何手段查看内容——黑盒。

因此这些用例的重点不是覆盖率，而是**钉死两条 API 路径都能工作**：
  1. 新路径（4.22+，GetMessageClass）
  2. 旧路径（<4.22，MessageFactory().GetPrototype()）

在开发机上通常只跑得到新路径，所以旧路径用 monkeypatch 强制走一遍——否则
这个测试会在开发机上全绿、在部署机上继续漏掉同一个 bug。

运行：
    cd src/amr_dispatcher_proto_cli && python3 -m pytest test/ -v
不需要 ROS 运行时，只需要 python3-protobuf。
"""
import sys
from pathlib import Path

import pytest

# 允许直接 `pytest test/` 而不用先安装包。
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from google.protobuf import descriptor_pb2
from google.protobuf import descriptor_pool
from google.protobuf import message_factory

from amr_dispatcher_proto_cli import descriptor_pool as dp


def _make_pool_with_one_message():
    """构造一个只含单条消息的最小描述符池。

    不依赖仓库里的 .proto 文件：那些文件的位置在安装前后不同（源码树 vs
    share/），把它们卷进单测会让「测试为什么失败」这个问题变成环境问题。
    这里手工搭 FileDescriptorProto，测的是**构造函数本身**，与具体 schema 无关。
    """
    fdp = descriptor_pb2.FileDescriptorProto()
    fdp.name = "test_probe.proto"
    fdp.package = "test.probe"
    fdp.syntax = "proto3"

    msg = fdp.message_type.add()
    msg.name = "Probe"
    field = msg.field.add()
    field.name = "value"
    field.number = 1
    field.type = descriptor_pb2.FieldDescriptorProto.TYPE_INT32
    field.label = descriptor_pb2.FieldDescriptorProto.LABEL_OPTIONAL

    pool = descriptor_pool.DescriptorPool()
    pool.Add(fdp)

    fds = descriptor_pb2.FileDescriptorSet()
    fds.file.append(fdp)
    return pool, fds


def test_message_class_for_returns_usable_class():
    """两条 API 路径都要能产出可实例化、可序列化往返的消息类。"""
    pool, _ = _make_pool_with_one_message()
    descriptor = pool.FindMessageTypeByName("test.probe.Probe")

    cls = dp._message_class_for(descriptor)

    msg = cls()
    msg.value = 1234
    raw = msg.SerializeToString()

    back = cls.FromString(raw)
    assert back.value == 1234


def test_legacy_path_without_get_message_class(monkeypatch):
    """强制走旧路径：模拟 protobuf < 4.22（Ubuntu 24.04 的实际版本）。

    这是回归测试的核心。做法是把 GetMessageClass 从模块上摘掉——正是
    `getattr(message_factory, 'GetMessageClass', None)` 这行探测代码要处理的
    情形。若有人把实现改回直接调用 `message_factory.GetMessageClass(...)`，
    这个用例会立刻失败。
    """
    monkeypatch.delattr(message_factory, "GetMessageClass", raising=False)
    assert not hasattr(message_factory, "GetMessageClass"), (
        "monkeypatch 没生效，这个用例就失去意义了"
    )

    pool, _ = _make_pool_with_one_message()
    descriptor = pool.FindMessageTypeByName("test.probe.Probe")

    cls = dp._message_class_for(descriptor)

    msg = cls()
    msg.value = 7
    assert cls.FromString(msg.SerializeToString()).value == 7


def test_legacy_path_uses_descriptors_own_pool():
    """旧路径必须传描述符**自己的**池，不能用默认全局池。

    用错池的典型报错是 "descriptor ... is not in the same pool"。这个约束
    很隐蔽：MessageFactory() 无参构造用的是默认池，而我们的描述符来自
    `_build_pool` 自建的池，两者不通用。
    """
    pool, _ = _make_pool_with_one_message()
    descriptor = pool.FindMessageTypeByName("test.probe.Probe")

    # 先确认默认池里没有这个类型——否则下面的断言可能因为「碰巧也在默认池里」
    # 而通过，测不出真正的约束。
    with pytest.raises(KeyError):
        descriptor_pool.Default().FindMessageTypeByName("test.probe.Probe")

    cls = dp._message_class_for(descriptor)
    assert cls().SerializeToString() == b""


def test_missing_both_apis_raises_actionable_error(monkeypatch):
    """两条 API 都不存在时，错误信息要能指导排查，而不是抛 AttributeError。

    牺牲可读性换来的「省几行」不值得：这个包的失败现场通常是部署后的
    运维环境，错误信息本身就是唯一线索。
    """
    monkeypatch.delattr(message_factory, "GetMessageClass", raising=False)
    monkeypatch.delattr(message_factory, "MessageFactory", raising=False)

    pool, _ = _make_pool_with_one_message()
    descriptor = pool.FindMessageTypeByName("test.probe.Probe")

    with pytest.raises(dp.ProtoSchemaError) as excinfo:
        dp._message_class_for(descriptor)

    message = str(excinfo.value)
    assert "GetMessageClass" in message
    assert "MessageFactory" in message
    # 提示必须给出下一步动作，否则运维看到「不支持」也只能干瞪眼。
    assert "python3-protobuf" in message
