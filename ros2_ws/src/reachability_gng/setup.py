import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'reachability_gng'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml')) +
            glob(os.path.join('config', '*.rviz'))),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='Raditya',
    maintainer_email='raditya.artha@gmail.com',
    description='GNG reachability/capability map for the redundant arm+table system.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Offline pipeline (run as plain python or via ros2 run)
            'data_gen = reachability_gng.data_gen:main',
            'train = reachability_gng.train:main',
            'eval = reachability_gng.eval:main',
            # Online ROS 2 nodes
            'seed_server = reachability_gng.seed_server:main',
            'seed_ik = reachability_gng.seed_ik:main',
            'visualize = reachability_gng.visualize:main',
            'object_localizer = reachability_gng.object_localizer:main',
            'seg_colorizer = reachability_gng.seg_colorizer:main',
            'seg_cloud = reachability_gng.seg_cloud:main',
            'reachability_check = reachability_gng.reachability_check:main',
            'reachability_cloud = reachability_gng.reachability_cloud:main',
            'collision_cloud = reachability_gng.collision_cloud:main',
            'infill_cloud = reachability_gng.infill_cloud:main',
            'table_slab = reachability_gng.table_slab:main',
            'object_collision = reachability_gng.object_collision:main',
            'octomap_refresher = reachability_gng.octomap_refresher:main',
            'map_static = reachability_gng.map_static:main',
            'static_collision = reachability_gng.static_collision:main',
            'gantry_reach_executor = '
            'reachability_gng.gantry_reach_executor:main',
            'pick_cli = reachability_gng.pick_cli:main',
        ],
    },
)
