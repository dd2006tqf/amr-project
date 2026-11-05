#!/usr/bin/env python3
"""从已编译的 FileDescriptorSet 生成 typesupport 绑定清单。

为什么不用正则扫 .proto 文本
---------------------------
正则扫描会漏掉嵌套消息、无法处理注释里的 `message` 字样，也无法知道
最终生成的 C++ 类名。改用 protoc 输出的 FileDescriptorSet——它已经是
protoc 解析后的权威结构，嵌套关系、包名、消息名都是准确的。

输出格式（供 py_typesupport_module.cpp include）：
    AMR_PROTO_BIND_TYPESUPPORT(amr_dispatcher::proto::ChassisLinkHealth,
                               type_support_chassis_link_health)
    ...

C++ 类名规则与 protoc 的 cpp 生成器一致：
  包名 a.b  →  命名空间 a::b
  消息 Nest.Outer  →  类 a::b::Outer_Nest（C++ 里嵌套消息用下划线连接）

用法：
    gen_typesupport_bindings.py <descriptors.pb> <output.inc> [--header X]
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from google.protobuf import descriptor_pb2


def cpp_type_name(package: str, cpp_name_parts: list[str]) -> str:
    """拼出 protoc 生成的 C++ 全限定类名。"""
    ns = '::'.join(package.split('.')) if package else ''
    cls = '_'.join(cpp_name_parts)
    return f'{ns}::{cls}' if ns else cls


def snake(name: str) -> str:
    """CamelCase -> snake_case，用于 Python 函数名。"""
    out: list[str] = []
    for i, ch in enumerate(name):
        if ch.isupper() and i > 0:
            out.append('_')
        out.append(ch.lower())
    return ''.join(out)


def walk(
    package: str,
    msg: descriptor_pb2.DescriptorProto,
    cpp_parts: list[str],
    acc: list[tuple[str, str]],
) -> None:
    parts = [*cpp_parts, msg.name]
    acc.append((cpp_type_name(package, parts), snake('_'.join(parts))))

    for nested in msg.nested_type:
        # map entry 是 protoc 合成的内部类型，不该暴露。
        if nested.options.map_entry:
            continue
        walk(package, nested, parts, acc)


def collect(fds: descriptor_pb2.FileDescriptorSet) -> list[tuple[str, str]]:
    acc: list[tuple[str, str]] = []
    for fdp in fds.file:
        # 只处理本项目自己的 proto，跳过 google/protobuf/* 之类的导入。
        if fdp.package != 'amr_dispatcher.proto':
            continue
        for msg in fdp.message_type:
            walk(fdp.package, msg, [], acc)
    # 去重并稳定排序，保证增量构建时输出稳定（避免无谓重编译）
    return sorted(set(acc))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('descriptor_set', type=Path)
    ap.add_argument('output', type=Path)
    ap.add_argument(
        '--header',
        type=Path,
        default=None,
        help='同时输出一份 C++ 头，声明所有绑定函数（供测试或外部包含）',
    )
    args = ap.parse_args()

    fds = descriptor_pb2.FileDescriptorSet()
    fds.ParseFromString(args.descriptor_set.read_bytes())

    entries = collect(fds)
    if not entries:
        print('错误: 没有在描述符集里找到 amr_dispatcher.proto 包的消息', file=sys.stderr)
        return 1

    lines = [
        '// 本文件由 gen_typesupport_bindings.py 自动生成，请勿手工编辑。',
        '// 源: FileDescriptorSet',
        '',
    ]
    for cpp_name, py_fn in entries:
        lines.append(f'AMR_PROTO_BIND_TYPESUPPORT({cpp_name}, {py_fn})')
    lines.append('')

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text('\n'.join(lines), encoding='utf-8')

    if args.header:
        hdr = [
            '// 本文件由 gen_typesupport_bindings.py 自动生成，请勿手工编辑。',
            '#pragma once',
            '',
        ]
        for cpp_name, py_fn in entries:
            hdr.append(
                f'PyObject * {py_fn}(const rosidl_message_type_support_t *);'
            )
        hdr.append('')
        args.header.parent.mkdir(parents=True, exist_ok=True)
        args.header.write_text('\n'.join(hdr), encoding='utf-8')

    print(f'生成 {len(entries)} 个 typesupport 绑定 -> {args.output}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
