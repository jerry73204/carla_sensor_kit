#!/usr/bin/env python3
"""Test CARLA connection and spawn a simple vehicle."""

import carla
import sys
import time

def main():
    try:
        # Connect to CARLA
        client = carla.Client('localhost', 2000)
        client.set_timeout(5.0)
        
        # Get world
        world = client.get_world()
        print(f"Connected to CARLA! Map: {world.get_map().name}")
        
        # Get blueprint library
        blueprint_library = world.get_blueprint_library()
        
        # Find a vehicle blueprint
        vehicle_bp = blueprint_library.filter('vehicle.*')[0]
        print(f"Selected vehicle: {vehicle_bp.id}")
        
        # Get spawn points
        spawn_points = world.get_map().get_spawn_points()
        if not spawn_points:
            print("No spawn points found!")
            return
            
        # Spawn vehicle
        spawn_point = spawn_points[0]
        vehicle = world.spawn_actor(vehicle_bp, spawn_point)
        print(f"Spawned vehicle at {spawn_point.location}")
        
        # Wait a bit
        time.sleep(5)
        
        # Cleanup
        vehicle.destroy()
        print("Vehicle destroyed")
        
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)

if __name__ == '__main__':
    main()