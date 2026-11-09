---
name: protobuf-typesupport-jazzy-constraints
description: protobuf 接入 ROS 2 话题时 Jazzy 的四个强制 typesupport 回调与 CDR 封装头偏移，改相关代码前必读
metadata: 
  node_type: memory
  type: project
  originSessionId: 8bec94ce-ce79-4480-83b7-6200b7629bfb
  modified: 2026-11-05T13:59:59.786Z
---

`amr_dispatcher_proto` 用零 fork 的 typesupport 注入让 protobuf 消息当 ROS 2 话题类型用。Jazzy（不像 Humble）有两个硬约束，二者都不看源码发现不了：

**1. 四个强制回调，缺一即崩**
`rcl` 无条件解引用这些函数指针：
- `get_type_hash_func` — `rcl/publisher.c:133`，缺则 SEGFAULT（跳 0x0）
- `get_type_description_func` — 同上
- `get_type_description_sources_func` — 同上
- introspection（经 `ts.func` 分发）— `TypeSupport_impl.cpp:612` `register_type_object`，缺则报 `failed to register type object with incompatible type`

注释里提到的这些源码位置是**实际核对过的**，不是推测。上游 ROS2-Protobuf 补丁无法改名移植的根因就在这：不是重命名，而是多了一层必须实现的协议。

introspection 合成结构只填名称与大小、不填字段偏移是安全的——`register_type_object` 只读名称做反查，从不读 `offset_` 或数组访问器。但代价是反射信息不完整，直接导致 `ros2 topic echo` / rqt / RViz 不可用。

**2. DDS 载荷前 4 字节不是本方案的**
```
[0,2) encapsulation kind  00 01 = CDR_LE / 00 00 = CDR_BE   ← Fast DDS 加的
[4,8) uint32 长度前缀                                        ← 本方案 cdr_serialize 写的
[8,..) protobuf 字节
```
解析端若只跳过 4 字节，会把 `00 01 00 00`（小端 256）当长度，**每帧都报"载荷过短"**。此坑已在 `echo.py` 的 `_strip_length_prefix` 注释中记录。

**Why:** 这两点是「看起来能跑但一定炸」的类型。理解原理无法推导出它们，只有读 rcl/rmw 源码或实际踩过才知道。

**How to apply:** 修改 `protobuf_typesupport.hpp` 或任何 protobuf 解析端前先看这条。升级 ROS 发行版时，先重新核对 `/opt/ros/<distro>/include/rcl/` 下的无条件解引用点是否新增。相关文档：仓库根目录 `PROTOBUF_MIGRATION_PLAN.md` §2.2、§2.3。
