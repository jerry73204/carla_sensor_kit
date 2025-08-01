from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'carla_sensor_kit_utils'

# Create wrapper scripts that will be installed to bin/
console_scripts = [
    'spawn_vehicle = carla_sensor_kit_utils.spawn_vehicle:main',
    'reset_carla = carla_sensor_kit_utils.reset_carla:main',
    'vehicle_monitor = carla_sensor_kit_utils.vehicle_monitor:main',
    'carla_package_generator = carla_sensor_kit_utils.package_generator:main',
    'carla_ros_bridge = carla_sensor_kit_utils.carla_ros_bridge:main',
]

setup(
    name=package_name.replace('_', '-'),  # setuptools prefers hyphens
    version='0.1.0',
    packages=find_packages(),
    package_data={
        package_name: [
            'templates/**/*',
            'templates/**/**/*',
            'templates/**/**/**/*',
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # Install default config file
        ('share/' + package_name + '/config', ['config/spawn_vehicle_config.yaml']),
        # Install executable scripts to bin directory
        ('bin', ['scripts/spawn_vehicle', 'scripts/reset_carla', 'scripts/vehicle_monitor', 'scripts/carla_package_generator', 'scripts/carla_ros_bridge']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='CARLA Team',
    maintainer_email='carla@example.com',
    description='Utilities for CARLA sensor kit integration',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': console_scripts,
    },
)