from setuptools import find_packages, setup

package_name = "seeker_media"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/media.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="seeker",
    maintainer_email="todo@todo.com",
    description="MP4 media player node for Seeker robot",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "mp4_player = seeker_media.mp4_player_node:main",
        ],
    },
)
