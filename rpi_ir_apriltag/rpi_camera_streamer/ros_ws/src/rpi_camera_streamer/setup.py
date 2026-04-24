from setuptools import setup
from glob import glob
import os

package_name = "rpi_camera_streamer"

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
            "camera_info_publisher = rpi_camera_streamer.camera_info_publisher:main",
            "webcam_watchdog = rpi_camera_streamer.webcam_watchdog:main",
        ],
    },
    install_requires=["setuptools", "pyyaml"],
    zip_safe=True,
    maintainer="you",
    maintainer_email="you@example.com",
    description="Camera streaming node for Raspberry Pi",
    license="MIT",
)
