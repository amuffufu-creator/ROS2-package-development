
import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'agv_base_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mrhaha',
    maintainer_email='mrhaha@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'udp_bridge_node = agv_base_controller.udp_bridge_node:main',
            'simple_goto_controller = agv_base_controller.simple_goto_controller:main',
            'imu_udp_bridge = agv_base_controller.imu_udp_bridge:main',
            'log_odom_speed = agv_base_controller.log_odom_speed:main',
        ],
    },
)
