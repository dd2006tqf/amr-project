from setuptools import find_packages
from setuptools import setup

package_name = 'amr_dispatcher_proto_cli'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tanqf',
    maintainer_email='tanqf@example.com',
    description='ros2 CLI extension for inspecting protobuf-encoded ROS topics',
    license='MIT',
    tests_require=['pytest'],
    # 两个扩展点：
    #   ros2cli.command      -> 提供 `ros2 amr` 顶层命令
    #   amr.verb             -> 提供 `ros2 amr echo` 子命令
    #   ros2cli.extension_point -> 声明 amr.verb 这个自定义扩展点
    entry_points={
        'ros2cli.command': [
            'amr = amr_dispatcher_proto_cli.command:AmrCommand',
        ],
        # 声明自定义扩展点 amr.verb，值为其基类（对应 ros2topic 的写法）
        'ros2cli.extension_point': [
            'amr.verb = amr_dispatcher_proto_cli.verb:VerbExtension',
        ],
        'amr.verb': [
            'echo = amr_dispatcher_proto_cli.echo_verb:EchoVerb',
        ],
    },
)
