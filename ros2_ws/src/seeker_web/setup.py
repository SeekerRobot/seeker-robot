from setuptools import find_packages, setup
from glob import glob

package_name = "seeker_web"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.py")),
        (f"share/{package_name}/static", glob("seeker_web/static/*")),
    ],
    include_package_data=True,
    package_data={package_name: ["static/*"]},
    install_requires=["setuptools", "aiohttp"],
    zip_safe=True,
    maintainer="seeker",
    maintainer_email="todo@todo.com",
    description="Browser-based all-in-one controller for the Seeker robot",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "web_node = seeker_web.web_node:main",
        ],
    },
)
