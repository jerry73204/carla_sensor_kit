#!/usr/bin/env python3
"""
Integrated version of simple_spawn.py that uses sensor configurations from YAML files.
This script spawns vehicles with sensors in CARLA using positions from Autoware sensor kit.
"""

import sys
import carla
import random
import queue
import logging
from collections import deque
from pathlib import Path

# Add the original carla_agent path for imports
sys.path.insert(0, "/home/aeon/repos/autoware_carla_launch/external/zenoh_carla_bridge/carla_agent")

# Add current directory for sensor_config_loader
sys.path.insert(0, str(Path(__file__).parent))

# Import sensor config loader
from sensor_config_loader import SensorConfigLoader

# Import original modules from carla_agent
from simulation.sensors import Lidar, SemanticLidar, Gnss, Imu, RgbCamera
from traffic_light import TrafficLightHelper
from global_route_planner import GlobalRoutePlanner


def main():
    """Main function that spawns vehicles with sensors using YAML configurations."""
    
    # Setup logging
    logging.basicConfig(level=logging.INFO)
    logger = logging.getLogger(__name__)
    
    # Connect to CARLA
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    
    logger.info("Connected to CARLA server")
    
    # Create sensor configuration loader
    try:
        config_loader = SensorConfigLoader()
        logger.info("Loaded sensor configurations from YAML files")
    except Exception as e:
        logger.error(f"Failed to load sensor configurations: {e}")
        return
    
    # Get spawn points
    spawn_points = world.get_map().get_spawn_points()
    
    # Dictionary to store actors
    actor_dict = {}
    
    try:
        # Spawn vehicle
        blueprint_library = world.get_blueprint_library()
        vehicle_bp = blueprint_library.filter('vehicle.*')[0]
        
        # Use first spawn point or random
        spawn_point = spawn_points[0] if spawn_points else carla.Transform()
        vehicle = world.spawn_actor(vehicle_bp, spawn_point)
        
        logger.info(f"Spawned vehicle: {vehicle.type_id} at {spawn_point.location}")
        actor_dict['vehicle'] = vehicle
        
        # Create sensor queue for data
        sensor_queue = queue.Queue()
        
        # Create sensors
        sensors = {
            'ublox': Gnss('ublox'),
            'tamagawa': Imu('tamagawa'),
            'top': Lidar('top'),
            'traffic_light': RgbCamera('traffic_light')
        }
        
        # Spawn sensors with transforms from YAML configuration
        for sensor_name, sensor in sensors.items():
            try:
                # Get transform from config based on sensor type
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
                
                # Spawn sensor with the loaded transform
                sensor_actor = sensor.spawn(world, vehicle, trans)
                sensor.listen(sensor_queue)
                actor_dict[sensor_name] = sensor_actor
                
                if trans:
                    logger.info(f"Spawned {sensor_name} at Location({trans.location.x:.2f}, "
                              f"{trans.location.y:.2f}, {trans.location.z:.2f})")
                else:
                    logger.info(f"Spawned {sensor_name} with default transform")
                    
            except Exception as e:
                logger.error(f"Failed to spawn sensor {sensor_name}: {e}")
        
        # Set vehicle to autopilot for testing
        vehicle.set_autopilot(True)
        logger.info("Vehicle set to autopilot mode")
        
        # Main loop
        logger.info("Starting main loop. Press Ctrl+C to stop.")
        
        while True:
            # Process sensor data
            try:
                data = sensor_queue.get(timeout=1.0)
                # Here you would normally process the sensor data
                # For now, we just acknowledge receipt
            except queue.Empty:
                pass
            
            # Tick world
            world.tick()
            
    except KeyboardInterrupt:
        logger.info("Stopping...")
    except Exception as e:
        logger.error(f"Error in main loop: {e}")
    finally:
        # Cleanup
        logger.info("Destroying actors...")
        for actor_name, actor in actor_dict.items():
            if actor is not None:
                actor.destroy()
                logger.info(f"Destroyed {actor_name}")
        
        logger.info("Cleanup complete")


if __name__ == '__main__':
    main()