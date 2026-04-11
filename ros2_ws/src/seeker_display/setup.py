from setuptools import find_packages, setup

package_name = "seeker_display"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="seeker",
    maintainer_email="todo@todo.com",
    description="OLED display nodes for Seeker robot",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "oled_sine = seeker_display.oled_sine_node:main",
        ],
    },
)
