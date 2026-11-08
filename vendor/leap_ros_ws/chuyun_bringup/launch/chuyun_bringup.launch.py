import os

from launch import LaunchDescription
from launch.actions import TimerAction

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    bringup_share = FindPackageShare(
        package='chuyun_bringup'
    ).find('chuyun_bringup')

    # 1. 获取机器人 URDF
    description_package = 'chuyun_description'
    urdf_name = 'chuyun.urdf'

    description_share = FindPackageShare(
        package=description_package
    ).find(description_package)

    urdf_model_path = os.path.join(
        description_share,
        'urdf',
        urdf_name
    )

    # 2. robot_state_publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        arguments=[urdf_model_path],
        output='screen'
    )

    # 3. joint_state_publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        arguments=[urdf_model_path],
        output='screen'
    )

    # 4. 为 MCU 传感器数据补齐协方差，再输入 EKF
    ekf_input_adapter_node = Node(
        package='chuyun_bringup',
        executable='ekf_input_adapter',
        name='ekf_input_adapter',
        output='screen'
    )

    # 5. 融合轮式里程计与 IMU，独占发布 odom -> base_footprint TF
    ekf_filter_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[os.path.join(bringup_share, 'config', 'ekf.yaml')],
        remappings=[('odometry/filtered', '/odometry/filtered')],
        output='screen'
    )

    # micro-ROS 代理和时间同步由 systemd 先行保证，再稍延时启动融合节点。
    delayed_nodes = TimerAction(
        period=2.0,
        actions=[
            joint_state_publisher_node,
            robot_state_publisher_node,
            ekf_input_adapter_node,
            ekf_filter_node,
        ]
    )

    return LaunchDescription([
        delayed_nodes,
    ])
