#!/usr/bin/env python3
"""
Simple script to spawn a vehicle with sensors in CARLA using the ROS 2 node.
This is a convenience wrapper around the carla_sensor_kit_utils vehicle_spawner node.
"""

import subprocess
import sys
import argparse
import os


def main():
    parser = argparse.ArgumentParser(description='Spawn a vehicle with sensors in CARLA')
    parser.add_argument('--host', default='localhost', help='CARLA server host')
    parser.add_argument('--port', default='2000', help='CARLA server port')
    parser.add_argument('--vehicle-type', default='vehicle.tesla.model3', help='Vehicle blueprint')
    parser.add_argument('--vehicle-name', default='hero', help='Vehicle role name')
    parser.add_argument('--spawn-point', default='-1', help='Spawn point index (-1 for random)')
    parser.add_argument('--no-autopilot', action='store_true', help='Disable autopilot')
    parser.add_argument('--no-sensors', action='store_true', help='Disable sensor spawning')
    
    args = parser.parse_args()
    
    # Build ROS command
    cmd = [
        'ros2', 'launch', 'carla_sensor_kit_utils', 'vehicle_spawner.launch.py',
        f'host:={args.host}',
        f'port:={args.port}',
        f'vehicle_type:={args.vehicle_type}',
        f'vehicle_role_name:={args.vehicle_name}',
        f'spawn_point_index:={args.spawn_point}',
        f'autopilot:={not args.no_autopilot}',
        f'enable_sensors:={not args.no_sensors}'
    ]
    
    print(f"Spawning vehicle '{args.vehicle_name}' in CARLA...")
    print(f"Command: {' '.join(cmd)}")
    
    try:
        # Source ROS environment and run
        source_cmd = 'source /opt/ros/humble/setup.bash && '
        if os.path.exists('install/setup.bash'):
            source_cmd += 'source install/setup.bash && '
        
        full_cmd = source_cmd + ' '.join(cmd)
        subprocess.run(full_cmd, shell=True, check=True)
    except KeyboardInterrupt:
        print("\nStopping vehicle spawner...")
    except subprocess.CalledProcessError as e:
        print(f"Error running spawner: {e}")
        sys.exit(1)


if __name__ == '__main__':
    main()