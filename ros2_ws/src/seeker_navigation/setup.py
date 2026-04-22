from setuptools import find_packages, setup
import os
from glob import glob

package_name = "seeker_navigation"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="seeker",
    maintainer_email="todo@todo.com",
    description="Ball search and navigation for the Seeker hexapod robot",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "ball_searcher = seeker_navigation.ball_searcher:main",
            "object_seeker = seeker_navigation.object_seeker:main",
            "scan_tilt_filter = seeker_navigation.scan_tilt_filter:main",
            "dead_reckoning_odom = seeker_navigation.dead_reckoning_odom:main",
            "cmd_vel_restrict = seeker_navigation.cmd_vel_restrict:main",
            "scripted_cmd_vel = seeker_navigation.scripted_cmd_vel:main",
            "find = seeker_navigation.find:main",
        ],
    },
)
