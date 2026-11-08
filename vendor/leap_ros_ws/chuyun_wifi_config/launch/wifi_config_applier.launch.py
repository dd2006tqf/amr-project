from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="chuyun_wifi_config",
            executable="wifi_config_applier",
            name="wifi_config_applier",
            output="screen",
            parameters=[
                {
                    "topic": "/wifi_config",
                    "interface": "wlan0",
                    "host_info_topic": "/host_info",
                    "host_info_repeats": 8,
                    "host_info_interval_sec": 1.0,
                    "host_info_route_target": "192.168.31.1",
                    "publish_host_info_on_startup": True,
                    "autoconnect": True,
                }
            ],
        )
    ])
