from setuptools import find_packages, setup
from glob import glob

package_name = "seeker_vision"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.py")),
        (f"share/{package_name}/model", ["model/yolo26n.pt"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="seeker",
    maintainer_email="todo@todo.com",
    description="Object detection and camera proxy nodes for the Seeker robot",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "vision_node = seeker_vision.vision_core:main",
            "gazebo_vision_node = seeker_vision.gazebo_vision_core:main",
            "cam_proxy = seeker_vision.cam_proxy:main",
        ],
    },
)
