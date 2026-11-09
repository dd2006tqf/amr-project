"""从已安装的 .proto 文件动态构建 protobuf 描述符池。

为什么需要这个
--------------
C++ 侧发出的字节是**标准 protobuf wire format**，跨语言通用。但 Python 侧
要解析它，必须先有一份 message 定义。ROS 的 `ros2 topic echo` 依赖 Python
与 C++ 使用同一份生成代码，而 protobuf 类型没有对应的 ROS Python 生成包。

本模块的做法：扫描 `share/amr_dispatcher_proto/proto/` 下的 .proto，用
`grpc_tools.protoc` 或系统 protoc 编译成 FileDescriptorProto，再构建
DescriptorPool，最后用 `message_factory` 动态生成 Python 类。

这样做的代价与边界：
  - **不需要** 为每个 proto 单独写 Python 生成规则
  - **需要** .proto 文件随包安装（CMakeLists 里已 install）
  - 首次构建描述符池有一次性开销（几十毫秒量级），随后缓存

依赖：
  - protobuf (Python 运行时)
  - 构建描述符池需要 protoc；若系统没有 protoc，退化为读取预编译的
    描述符集文件（由 CMake 在构建期生成，见 CMakeLists 中 proto_descriptors 目标）
"""

from __future__ import annotations

import os
import subprocess
import tempfile
from pathlib import Path
from typing import Dict, Optional

from google.protobuf import descriptor_pb2
from google.protobuf import descriptor_pool
from google.protobuf import message_factory


class ProtoSchemaError(RuntimeError):
    """无法构建 protobuf 描述符池时抛出。"""


def _find_descriptor_set() -> Optional[Path]:
    """定位 CMake 构建期生成的 descriptors.pb。

    这是首选路径：它是 protoc 在构建时对同一批 .proto 的解析结果，
    与 C++ 侧用的是同一份权威数据，且**运行期不需要 protoc**。
    """
    try:
        from ament_index_python.packages import get_package_share_directory
        candidate = (
            Path(get_package_share_directory('amr_dispatcher_proto'))
            / 'proto' / 'descriptors.pb'
        )
        if candidate.is_file():
            return candidate
    except Exception:  # noqa: BLE001 - ament_index 在非 ROS 环境会抛多种异常
        pass

    # 开发态回退：build/ 目录里找
    here = Path(__file__).resolve()
    for parent in here.parents:
        for cand in parent.glob('build/amr_dispatcher_proto/generated/descriptors.pb'):
            if cand.is_file():
                return cand
    return None


def _find_proto_dir() -> Path:
    """定位已安装的 .proto 目录。

    优先用 ament_index 查 share 路径；找不到再退回相对路径猜测。
    """
    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory('amr_dispatcher_proto'))
        candidate = share / 'proto'
        if candidate.is_dir():
            return candidate
    except Exception:  # noqa: BLE001 - ament_index 在非 ROS 环境会抛多种异常
        pass

    # 开发态回退：源码树内直接运行
    here = Path(__file__).resolve()
    for parent in here.parents:
        candidate = parent / 'proto'
        if candidate.is_dir() and any(candidate.glob('*.proto')):
            return candidate

    raise ProtoSchemaError(
        'amr_dispatcher_proto/proto 目录未找到。'
        '请确认 amr_dispatcher_proto 包已安装（share/amr_dispatcher_proto/proto）。'
    )


def _compile_protos(proto_dir: Path) -> descriptor_pb2.FileDescriptorSet:
    """用 protoc 把 .proto 编译成 FileDescriptorSet。

    优先使用 protoc 可执行文件。若不可用，尝试 grpc_tools.protoc（纯 Python
    实现，随 grpcio-tools 安装）。
    """
    proto_files = sorted(proto_dir.glob('*.proto'))
    if not proto_files:
        raise ProtoSchemaError(f'{proto_dir} 下没有 .proto 文件')

    with tempfile.TemporaryDirectory() as tmpdir:
        out_file = Path(tmpdir) / 'descriptors.pb'

        cmd = [
            'protoc',
            f'--proto_path={proto_dir}',
            f'--descriptor_set_out={out_file}',
            '--include_imports',
            *[str(p) for p in proto_files],
        ]

        try:
            subprocess.run(cmd, check=True, capture_output=True)
        except FileNotFoundError:
            # 退化路径：grpc_tools 的 protoc 是 Python 模块
            try:
                from grpc_tools import protoc as grpc_protoc
            except ImportError as exc:
                raise ProtoSchemaError(
                    '系统没有 protoc，也没有 grpc_tools。'
                    '请安装 protobuf-compiler，或 pip install grpcio-tools。'
                ) from exc

            rc = grpc_protoc.main(cmd)
            if rc != 0:
                raise ProtoSchemaError(f'grpc_tools.protoc 返回 {rc}')
        except subprocess.CalledProcessError as exc:
            stderr = exc.stderr.decode('utf-8', 'replace') if exc.stderr else ''
            raise ProtoSchemaError(f'protoc 失败: {stderr}') from exc

        fds = descriptor_pb2.FileDescriptorSet()
        fds.ParseFromString(out_file.read_bytes())
        return fds


def _build_pool(fds: descriptor_pb2.FileDescriptorSet) -> descriptor_pool.DescriptorPool:
    """把 FileDescriptorSet 灌进 DescriptorPool。

    import 顺序问题：common.proto 被其余文件 import，必须先进池，否则
    后面 add() 会因依赖缺失失败。这里做拓扑排序——先加所有无依赖的，
    再迭代加可满足的，直到全部加入。
    """
    pool = descriptor_pool.DescriptorPool()
    pending = list(fds.file)

    # 已加入池子的文件名。注意：只有真正 pool.Add() 成功的才写进来。
    # 早期版本这里预置了 google/protobuf/* 的名字作为"视为已满足"，
    # 那是错的——依赖检查会误判为可满足，而该文件其实还没进池，
    # 后续按名字查类型就会找不到。--include_imports 保证被 import 的
    # 文件本身也在 fds 里，所以正常拓扑排序即可，无需特殊预置。
    added: set[str] = set()

    guard = 0
    while pending:
        guard += 1
        if guard > 1000:
            raise ProtoSchemaError('描述符依赖解析死循环，请检查 .proto 的 import 关系')

        progressed = False
        for fdp in list(pending):
            deps = list(fdp.dependency)
            if all(dep in added for dep in deps):
                try:
                    pool.Add(fdp)
                except Exception as exc:  # noqa: BLE001
                    # 重复添加是允许的（同一文件可能被多次 include）
                    if 'already' not in str(exc).lower():
                        raise ProtoSchemaError(
                            f'添加 {fdp.name} 到描述符池失败: {exc}'
                        ) from exc
                added.add(fdp.name)
                pending.remove(fdp)
                progressed = True

        if not progressed:
            unresolved = ', '.join(p.name for p in pending)
            missing = sorted({
                dep
                for p in pending
                for dep in p.dependency
                if dep not in added
            })
            raise ProtoSchemaError(
                f'以下 .proto 的依赖无法解析: {unresolved}\n'
                f'缺失的依赖: {missing}\n'
                '若缺失项是 google/protobuf/* 的 well-known types，'
                '请确认生成描述符集时带了 --include_imports。'
            )

    return pool


class SchemaRegistry:
    """protobuf 类型名 -> Python 消息类 的惰性缓存。"""

    def __init__(self) -> None:
        self._pool: Optional[descriptor_pool.DescriptorPool] = None
        self._classes: Dict[str, type] = {}
        self._all_names: list[str] = []

    def _ensure_pool(self) -> descriptor_pool.DescriptorPool:
        if self._pool is None:
            # 首选：CMake 构建期生成的描述符集，无需运行期 protoc。
            descriptor_set = _find_descriptor_set()
            if descriptor_set is not None:
                fds = descriptor_pb2.FileDescriptorSet()
                fds.ParseFromString(descriptor_set.read_bytes())
            else:
                # 回退：现场调用 protoc 编译 .proto。
                # 只在开发态（未构建 C++ 包）会走到这里。
                fds = _compile_protos(_find_proto_dir())

            self._pool = _build_pool(fds)
            # 注意：protobuf 4.x 的 C++ 实现（upb）不提供
            # DescriptorPool.FindAllMessageTypes()，那是纯 Python 实现才有的。
            # 所以这里从 FileDescriptorSet 自己收集，而不是问池子。
            self._all_names = _collect_message_names(fds)
        return self._pool

    def message_names(self) -> list[str]:
        """返回所有已编译的 protobuf 消息全名（已排序）。"""
        self._ensure_pool()
        return list(self._all_names)

    def get_class(self, full_name: str) -> type:
        """按完整类型名取 Python 消息类。

        full_name 形如 'amr_dispatcher.proto.ChassisLinkHealth'。
        """
        if full_name in self._classes:
            return self._classes[full_name]

        pool = self._ensure_pool()
        try:
            descriptor = pool.FindMessageTypeByName(full_name)
        except KeyError as exc:
            available = self._all_names
            raise ProtoSchemaError(
                f'类型 {full_name} 不在描述符池中。\n已知类型: {available}'
            ) from exc

        cls = message_factory.GetMessageClass(descriptor)
        self._classes[full_name] = cls
        return cls


def _collect_message_names(fds: descriptor_pb2.FileDescriptorSet) -> list[str]:
    """递归收集 FileDescriptorSet 里所有消息（含嵌套）的全名。"""
    names: list[str] = []

    def walk(prefix: str, msg: descriptor_pb2.DescriptorProto) -> None:
        full = f'{prefix}.{msg.name}' if prefix else msg.name
        names.append(full)
        for nested in msg.nested_type:
            walk(full, nested)

    for fdp in fds.file:
        pkg = fdp.package
        for msg in fdp.message_type:
            walk(pkg, msg)

    return sorted(names)


# 进程级单例：构建描述符池有开销，避免每条消息重复构建。
registry = SchemaRegistry()
