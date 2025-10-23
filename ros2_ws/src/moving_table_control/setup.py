from setuptools import setup, find_packages
import os
from glob import glob

package_name = "moving_table_control"

setup(
    name=package_name,
    version="0.0.0",
    
    #
    # --- THIS IS THE CRITICAL CHANGE ---
    # This will now automatically find BOTH your 'moving_table' library package
    # AND your 'moving_table_control' node package.
    #
    packages=find_packages(exclude=['test']),
    
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/moving_table_control"]),
        ("share/" + package_name, ["package.xml"]),
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
    
    # This entry point is now correct because 'dual_table_controller.py'
    # is inside the 'moving_table_control' package directory.
    entry_points={
        'console_scripts': [
            'dual_table_controller = moving_table_control.dual_table_controller:main',
        ],
    },
)
