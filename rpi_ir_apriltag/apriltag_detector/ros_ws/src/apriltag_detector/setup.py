from setuptools import setup
from glob import glob
import os

package_name = "apriltag_detector"

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
            "detection_overlay = apriltag_detector.detection_overlay:main",
            "field_localizer = apriltag_detector.field_localizer:main",
            "field_visualizer = apriltag_detector.field_visualizer:main",
            "robot_boundary_visualizer = apriltag_detector.robot_boundary_visualizer:main",
        ],
    },
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="you",
    maintainer_email="you@example.com",
    description="AprilTag detection from camera streams",
    license="MIT",
)
