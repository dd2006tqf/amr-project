from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory


def generate_launch_description():
    urdf_path = PathJoinSubstitution([
        FindPackageShare('chuyun_description'),
        'urdf',
        'chuyun.urdf',
    ])

    use_gui = LaunchConfiguration('use_gui')
    use_rviz = LaunchConfiguration('rviz')

    nodes = [
        DeclareLaunchArgument('use_gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': Command(['cat ', urdf_path])}],
            output='screen',
        )
    ]

    try:
        get_package_share_directory('joint_state_publisher_gui')
        nodes.append(Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            condition=IfCondition(use_gui),
            output='screen',
        ))
    except PackageNotFoundError:
        nodes.append(Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            output='screen',
        ))

    try:
        get_package_share_directory('rviz2')
        nodes.append(Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('chuyun_description'),
                'rviz',
                'display.rviz',
            ])],
            condition=IfCondition(use_rviz),
            output='screen',
        ))
    except PackageNotFoundError:
        pass

    return LaunchDescription(nodes)
