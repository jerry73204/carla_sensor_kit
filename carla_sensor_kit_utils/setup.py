from setuptools import setup
from glob import glob
import os

package_name = 'carla_sensor_kit_utils'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='CARLA Team',
    maintainer_email='carla@example.com',
    description='Utilities for CARLA sensor kit integration',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vehicle_spawner = carla_sensor_kit_utils.vehicle_spawner:main',
        ],
    },
)