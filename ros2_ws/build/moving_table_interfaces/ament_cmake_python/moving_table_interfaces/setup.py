from setuptools import find_packages
from setuptools import setup

setup(
    name='moving_table_interfaces',
    version='0.0.0',
    packages=find_packages(
        include=('moving_table_interfaces', 'moving_table_interfaces.*')),
)
