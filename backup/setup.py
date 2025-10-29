from setuptools import setup, find_packages
import os
from glob import glob

package_name = "moving_table_pkg"

setup(
    name=package_name,
    version="0.0.0",
    # This finds both your 'moving_table' lib and 'moving_table_pkg' node
    packages=find_packages(exclude=['test']),
    
    # This data_files section is required for pure Python packages
    # to install non-Python files (like launch, srv, xacro)
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Install all files from the launch directory
        (os.path.join('share', package_name, 'launch'), 
            glob(os.path.join('launch', '*.launch.py'))),
            
        # Install all files from the robots directory
        (os.path.join('share', package_name, 'robots'), 
            glob(os.path.join('robots', '*.xacro'))),
            
        # Install all .srv files
        (os.path.join('share', package_name, 'srv'), 
            glob(os.path.join('srv', '*.srv'))),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Your Name",
    maintainer_email="your.email@example.com",
    description="ROS2 package for controlling two moving tables",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        'console_scripts': [
            'dual_table_controller = moving_table_pkg.dual_table_controller:main',
        ],
    },
)
