from setuptools import setup
import os
from glob import glob

package_name = "moving_table_control"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name, "moving_table_control.moving_table"],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/moving_table_control"],
        ),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "srv"), glob("srv/*.srv")),
        (os.path.join("share", package_name, "robots"), glob("robots/*.xacro")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Your Name",
    maintainer_email="your.email@example.com",
    description="ROS2 package for controlling two moving tables",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "dual_table_controller = moving_table_control.dual_table_controller:main",
        ],
    },
)
