from glob import glob
from setuptools import find_packages, setup

package_name = "sayai_voice_sim"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="SayAI",
    maintainer_email="user@example.com",
    description="Lightweight voice command simulation package for robot task integration.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "voice_command_manager = sayai_voice_sim.voice_command_manager:main",
            "mock_robot_task_server = sayai_voice_sim.mock_robot_task_server:main",
            "real_robot_task_server = sayai_voice_sim.real_robot_task_server:main",
            "mobile_robot_task_server = sayai_voice_sim.mobile_robot_task_server:main",
            "whisper_transcript_bridge = sayai_voice_sim.whisper_transcript_bridge:main",
            "voice_web_ui = sayai_voice_sim.voice_web_ui:main",
        ],
    },
)
