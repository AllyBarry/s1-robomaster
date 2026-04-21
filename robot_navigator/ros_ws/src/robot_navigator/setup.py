from setuptools import setup
from glob import glob
import os

package_name = "robot_navigator"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    entry_points={
        "console_scripts": [
            "navigator = robot_navigator.navigator:main",
            "follower = robot_navigator.follower:main",
        ],
    },
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="you",
    maintainer_email="you@example.com",
    description="Per-robot goal-following navigator with dead-reckoning",
    license="MIT",
)
