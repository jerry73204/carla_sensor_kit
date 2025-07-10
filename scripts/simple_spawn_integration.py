#!/usr/bin/env python3
"""
Example showing how to integrate sensor_config_loader.py with the existing simple_spawn.py
This demonstrates the direct replacement method.
"""

import sys
from pathlib import Path

# Add paths for imports
sys.path.insert(0, str(Path(__file__).parent))
sys.path.insert(0, "/home/aeon/repos/autoware_carla_launch/external/zenoh_carla_bridge/carla_agent")

from sensor_config_loader import SensorConfigLoader
import carla


def modify_sensor_spawn(sensor, world, parent_actor, config_loader):
    """
    Modify the sensor spawn method to use configuration from YAML.
    
    This function shows how to override the default transform in the existing sensor classes.
    """
    # Get the appropriate transform based on sensor name
    if sensor.sensor_name == 'ublox':  # GNSS
        trans = config_loader.get_gnss_transform()
    elif sensor.sensor_name == 'tamagawa':  # IMU
        trans = config_loader.get_imu_transform()
    elif sensor.sensor_name == 'top':  # LiDAR
        trans = config_loader.get_lidar_transform('top')
    elif sensor.sensor_name == 'traffic_light':  # Camera
        trans = config_loader.get_camera_transform('traffic_light')
    else:
        # Use default transform
        trans = None
    
    # Call the original spawn method with our transform
    return sensor.spawn(world, parent_actor, trans)


# Example of modified simple_spawn.py section:
"""
# In simple_spawn.py, replace this section:

# Original code:
sensors = {
    'ublox': Gnss('ublox'),
    'tamagawa': Imu('tamagawa'),
    'top': Lidar('top'),
    'traffic_light': RgbCamera('traffic_light')
}

for sensor_name, sensor in sensors.items():
    sensor_actor = sensor.spawn(world, vehicle)
    sensor.listen(sensor_queue)
    actor_dict[sensor_name] = sensor_actor

# New code with config loader:
from sensor_config_loader import SensorConfigLoader

# Create config loader once
config_loader = SensorConfigLoader()

# Create sensors as before
sensors = {
    'ublox': Gnss('ublox'),
    'tamagawa': Imu('tamagawa'),
    'top': Lidar('top'),
    'traffic_light': RgbCamera('traffic_light')
}

# Spawn with transforms from YAML
for sensor_name, sensor in sensors.items():
    # Get transform from config
    if sensor_name == 'ublox':
        trans = config_loader.get_gnss_transform()
    elif sensor_name == 'tamagawa':
        trans = config_loader.get_imu_transform()
    elif sensor_name == 'top':
        trans = config_loader.get_lidar_transform('top')
    elif sensor_name == 'traffic_light':
        trans = config_loader.get_camera_transform('traffic_light')
    else:
        trans = None
    
    # Spawn with the transform
    sensor_actor = sensor.spawn(world, vehicle, trans)
    sensor.listen(sensor_queue)
    actor_dict[sensor_name] = sensor_actor
"""


def print_integration_steps():
    """Print step-by-step integration instructions."""
    print("Direct Integration Steps for simple_spawn.py:")
    print("=" * 60)
    print("\n1. Add import at the top of simple_spawn.py:")
    print("   from sensor_config_loader import SensorConfigLoader")
    
    print("\n2. Create config loader after connecting to CARLA:")
    print("   config_loader = SensorConfigLoader()")
    
    print("\n3. Modify the sensor spawning loop to use transforms from YAML:")
    print("""
   # Replace the sensor spawning loop with:
   for sensor_name, sensor in sensors.items():
       # Get transform from config
       if sensor_name == 'ublox':
           trans = config_loader.get_gnss_transform()
       elif sensor_name == 'tamagawa':
           trans = config_loader.get_imu_transform()
       elif sensor_name == 'top':
           trans = config_loader.get_lidar_transform('top')
       elif sensor_name == 'traffic_light':
           trans = config_loader.get_camera_transform('traffic_light')
       else:
           trans = None
       
       # Spawn with the transform
       sensor_actor = sensor.spawn(world, vehicle, trans)
       sensor.listen(sensor_queue)
       actor_dict[sensor_name] = sensor_actor
""")
    
    print("\n4. That's it! Sensors will now use positions from YAML files.")
    print("\nAdvantages:")
    print("  - Minimal code changes")
    print("  - Single source of truth for sensor positions")
    print("  - Easy to maintain and update")
    print("  - Consistent with Autoware sensor kit")


if __name__ == "__main__":
    print_integration_steps()
    
    print("\n" + "=" * 60)
    print("Testing sensor transforms...")
    
    try:
        loader = SensorConfigLoader()
        transforms = {
            'ublox': loader.get_gnss_transform(),
            'tamagawa': loader.get_imu_transform(),
            'top': loader.get_lidar_transform('top'),
            'traffic_light': loader.get_camera_transform('traffic_light')
        }
        
        print("\nTransforms ready for use:")
        for name, trans in transforms.items():
            print(f"  {name}: Location({trans.location.x:.2f}, {trans.location.y:.2f}, {trans.location.z:.2f})")
    except Exception as e:
        print(f"\nError: {e}")