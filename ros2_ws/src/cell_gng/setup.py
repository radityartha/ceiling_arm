import os
from glob import glob

from setuptools import find_packages, setup

package_name = "cell_gng"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="radityartha",
    maintainer_email="raditya.artha@gmail.com",
    description="GNG-U2-style topological mapping node for the workcell.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "gng_node = cell_gng.gng_node:main",
            "gng_validation_node = cell_gng.gng_validation_node:main",
            "fake_cloud_publisher = cell_gng.fake_cloud_publisher:main",
        ],
    },
)
