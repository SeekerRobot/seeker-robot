from glob import glob
from setuptools import find_packages, setup

package_name = "seeker_test_cmd"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="seeker",
    maintainer_email="todo@todo.com",
    description="Testing node to translate AI intents into cmd_vel commands",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "velocity_node = seeker_test_cmd.velocity_node:main",
        ],
    },
)