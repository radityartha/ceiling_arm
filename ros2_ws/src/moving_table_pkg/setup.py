from setuptools import setup, find_packages
import os
from glob import glob

package_name = "moving_table_pkg"

setup(
    name=package_name,
    version="0.0.0",
    # Use find_packages() to automatically discover your Python packages
    # (This will find 'moving_table' and 'moving_table_pkg')
    packages=find_packages(exclude=['test']),
    
    # This data_files section correctly installs your launch and robot files
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
    ],
    # This line fixes the "No module named 'pymodbus'" error
    install_requires=['setuptools', 'pymodbus'],
    zip_safe=True,
    maintainer="Your Name",
    maintainer_email="your.email@example.com",
    description="Python part of the moving_table_pkg",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        'console_scripts': [
            'dual_table_controller = moving_table_pkg.dual_table_controller:main',
        ],
    },
)