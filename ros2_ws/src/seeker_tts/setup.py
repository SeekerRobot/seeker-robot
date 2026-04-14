from setuptools import find_packages, setup
from glob import glob

package_name = "seeker_tts"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.py")),
        (f"share/{package_name}/sounds", glob("sounds/*.wav")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="seeker",
    maintainer_email="todo@todo.com",
    description="TTS bridge: /audio_transcription -> Fish Audio -> ESP32 speaker",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "tts_node = seeker_tts.tts_node:main",
        ],
    },
)
