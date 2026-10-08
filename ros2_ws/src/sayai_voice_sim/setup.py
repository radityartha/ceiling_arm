from setuptools import find_packages, setup

package_name = "sayai_voice_sim"

# DEMO-OFF (docs/p1_demo_off.md d2): only voice_web_ui was copied from the
# siap_demo checkout (/home/user1/ceiling_arm). The old task servers (hardcoded
# joints + grasp) and the Whisper bridge are deliberately NOT part of this repo.
setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="SayAI",
    maintainer_email="user@example.com",
    description="Browser mic -> /voice/transcript (voice_web_ui only).",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "voice_web_ui = sayai_voice_sim.voice_web_ui:main",
        ],
    },
)
