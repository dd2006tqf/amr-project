---
name: lower-layer-firmware-build-env
description: 下位机 ESP32-S3 固件已能在本机完整构建成功（2026-11-09），含 5 个必踩的坑与修法
metadata:
  node_type: memory
  type: project
  originSessionId: e1788e8c-1686-4b78-b185-8f8ed807d83c
  modified: 2026-11-09T05:18:02.690Z
---

下位机固件（`lower/leap_ros_mcu_driver/`，ESP32-S3）编译环境。**2026-11-09 已在两台机器上完整构建成功**，`leap_ros.bin` 1107456 字节（0x10e600），单条 `idf.py build` 即可。

**正确构建命令（一条即可，不用手动 set-target）**：
```bash
export IDF_SKIP_CHECK_SUBMODULES=1
export CMAKE_BUILD_PARALLEL_LEVEL=1     # 内存 < 8 GB 必须；见 [[low-memory-build-constraints]]
source <IDF 路径>/export.sh
cd lower/leap_ros_mcu_driver && idf.py build
```

**IDF 根目录随机器不同，不要写死**：
- 旧机器（`ubuntu@…`，4 核 / 3.7 GB）：`/home/ubuntu/esp/esp-idf`，干净构建 83–86 秒
- 本机 VMware `dev-cloud-m1`（CentOS Stream 10，2 vCPU / 7.5 GiB）：`/home/tanqf/esp/esp-idf`，干净构建 **388–467 秒**，no-op 增量 4 秒、改 1 个 TU 约 51 秒

耗时差 4.5 倍纯由核数决定，不是配置问题。**不要把这组速度数字当成通用基线。**

## 五个坑（全部已修，修法已入库）

**1. 22 个 IDF 子模块拉不全**（gitee 镜像不镜像子模块）。
已从 `codeload.github.com`（唯一可用 CDN，支持 `/tar.gz/<commit>`）下载全部 23 个子模块归档并解压进 `/home/ubuntu/esp/esp-idf`。
`git submodule status` 仍报 `-`（未初始化），但文件齐全；靠 `IDF_SKIP_CHECK_SUBMODULES=1` 跳过检查。
**不要相信 `idf.py --version` 能跑就等于能编译**——子模块缺失到 `set-target` 才暴露。

**2. `idf.py` 不支持 `-j`**。`build` 子命令没有该选项（`-j` 放在 `set-target`/`build` 前后都报 `No such option`）。限并发只能靠 `CMAKE_BUILD_PARALLEL_LEVEL=1`。

**3. `micro_ros_espidf_component/network_interfaces/` 目录上游漏提交**。
该目录在本仓库 git 历史里从未存在过，但 `CMakeLists.txt` 的 `INCLUDE_DIRS "network_interfaces"` 强制要求。已从上游 humble 分支补回 3 个文件（逐字节一致）。
主程序用自己的 `main/network/wifi_app.cpp`，**不调用** `uros_network_interface_initialize()`，所以补回不改变运行时行为。

**4. 组件的 CMakeLists 无条件重建 libmicroros**。
上游 `execute_process(make -f libmicroros.mk)` 每次配置都会 `rm -rf micro_ros_src` 并 git clone 整套 micro-ROS，再跑 colcon。
本机 `colcon: not found`，且入库的 `micro_ros_dev/install` 里 colcon 脚本硬编码 `/root/maturo_project/leap_low_v1/...`（上游作者机器）。
已在 `CMakeLists.txt` 外面包 `if(EXISTS libmicroros.a AND IS_DIRECTORY include)` 跳过重建，复用预编译产物（`libmicroros.a` 1133 个成员 + `include/` 2234 文件）。
同时注释掉 `add_dependencies(libmicroros-prebuilt libmicroros_project)`——`libmicroros_project` 目标上游从未定义。

**5. `dependencies.lock` 里是上游机器的绝对路径** `/root/maturo_project/...`。
`/root` 是 `drwx------`，组件管理器 `stat()` 时抛 `PermissionError: [Errno 13]`，`set-target`/`build` 全盘失败。
关键：**idf-component-manager 每次配置都会重写这份 lock，并把 local 依赖写成当前机器的绝对路径**——改成相对路径也会被改回去，改成别人机器的路径同样无意义。
**修法：`git rm --cached` + 加入 `.gitignore`**。`main/idf_component.yml` 已有相对路径 `path: ../managed_components/micro_ros_espidf_component`，缺失 lock 时组件管理器会按 manifest 重新生成。

## 两处配置钉死（`sdkconfig.defaults` + `CMakeLists.txt`）

- **target**：仓库不提交 `sdkconfig`。全新克隆后 `idf.py build` 会按 IDF 默认走 **esp32**，`battery_adc.cpp:51` 的 `adc_cali_curve_fitting_config_t` 在 esp32 上不存在 → 编译失败。
  已 `set(IDF_TARGET "esp32s3")` 在 `CMakeLists.txt` 的 `include(project.cmake)` **之前**，并在 `sdkconfig.defaults` 写 `CONFIG_IDF_TARGET="esp32s3"` 双保险。
- **分区表**：默认 `partitions_singleapp.csv` 的 factory 只有 1 MB，固件 1.06 MB 溢出 57 KB。
  已 `CONFIG_PARTITION_TABLE_SINGLE_APP_LARGE=y`（factory 1500K，余量 28%）。代价是放弃 OTA 双分区——本仓库当前无 OTA 流程。

**How to apply**：
- 改固件构建相关文件前先读这一条。五个坑的修法都有注释说明原因，不要"清理"掉那些 NOTE。
- `lower/leap_ros_mcu_driver/build/`、`sdkconfig`、`dependencies.lock` 都是本地产物，不入库。
- 低内存约束见 [[low-memory-build-constraints]]：3.7 GB 机器必须 `CMAKE_BUILD_PARALLEL_LEVEL=1`。
- 固件构建**不在任何 CI 里**（`scripts/` `docker/` `.github/` 均无 `idf.py` 调用），与 [[clean-build-only-failures]] 记录的 "CI 从不跑 colcon test" 是同一类缺口。

相关：[[low-memory-build-constraints]]、[[clean-build-only-failures]]、[[deploy-only-failure-modes]]
