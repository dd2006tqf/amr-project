from glob import glob
import os

from setuptools import find_packages, setup

package_name = "chuyun_wifi_config"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="xuegeros",
    maintainer_email="xuegeros@todo.todo",
    description="Apply Wi-Fi credentials from ROS 2 topic using nmcli.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "wifi_config_applier = chuyun_wifi_config.wifi_config_applier:main",
        ],
    },
)
