from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "use_test_image",
            default_value="true",
            description="Publish the built-in synthetic image for testing"),
        DeclareLaunchArgument(
            "input_topic",
            default_value="",
            description="Optional sensor_msgs/Image topic to process"),
        DeclareLaunchArgument(
            "publish_rate",
            default_value="5.0",
            description="Synthetic image publish rate in Hz"),
        Node(
            package="chuyun_opencv",
            executable="opencv_demo_node",
            name="opencv_demo_node",
            output="screen",
            parameters=[{
                "use_test_image": LaunchConfiguration("use_test_image"),
                "input_topic": LaunchConfiguration("input_topic"),
                "publish_rate": LaunchConfiguration("publish_rate"),
            }],
        ),
    ])
