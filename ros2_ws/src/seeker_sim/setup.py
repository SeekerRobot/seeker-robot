from setuptools import find_packages, setup

package_name = 'seeker_sim'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='seeker',
    maintainer_email='todo@todo.com',
    description='Simulation stand-in for the Seeker hexapod MCU',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'fake_mcu_node = seeker_sim.fake_mcu_node:main',
        ],
    },
)
