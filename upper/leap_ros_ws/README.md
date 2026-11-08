# chuyun_ws

Chuyun 机器人的 ROS 2 工作空间源码。仓库根目录对应工作空间的 `src` 目录。

## 工作空间包

- `chuyun_bringup`：底盘节点、里程计和 TF 处理及启动文件。
- `chuyun_camera`：HTTP/MJPEG 摄像头流发布节点。
- `chuyun_cartographer`：Cartographer 配置和启动文件。
- `chuyun_description`：机器人 URDF、模型和 RViz 资源。
- `chuyun_navigation2`：轻量定位与导航、Nav2 配置和地图。
- `chuyun_opencv`：颜色、形状和模板匹配示例。
- `chuyun_wifi_config`：通过 ROS 2 话题接收 Wi-Fi 配置并调用 nmcli。
- `leap_interfaces`：micro-ROS 固件使用的服务接口。
- `micro-ROS-Agent`、`micro_ros_msgs`：micro-ROS 代理及消息定义。
- `openslam_gmapping`、`slam_gmapping`：gmapping 源码和 ROS 2 封装。
- `ros2_astra_camera`：Astra 摄像头驱动源码。

## 构建

准备好 ROS 2 环境及各包所需依赖后：

```bash
mkdir -p ~/chuyun_ws
git clone https://github.com/czu963889306-dev/chuyun_ws.git ~/chuyun_ws/src
cd ~/chuyun_ws
colcon build
source install/setup.bash
```

只构建单个包：

```bash
colcon build --packages-select chuyun_description
```

## 资源位置

- 机器人描述：`chuyun_description/urdf/chuyun.urdf`。
- 视觉模型：`chuyun_description/meshes/`。
- 导航地图：`chuyun_navigation2/maps/`。
