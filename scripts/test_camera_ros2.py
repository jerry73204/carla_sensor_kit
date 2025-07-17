#!/usr/bin/env python3
"""Test script to debug camera ROS2 publishing in CARLA"""

import carla
import time
import sys

def main():
    try:
        # Connect to CARLA
        client = carla.Client('localhost', 3000)
        client.set_timeout(10.0)
        world = client.get_world()
        
        # Get a spawn point
        spawn_points = world.get_map().get_spawn_points()
        spawn_point = spawn_points[0]
        
        # Spawn a vehicle
        bp_lib = world.get_blueprint_library()
        vehicle_bp = bp_lib.filter('vehicle.*')[0]
        vehicle_bp.set_attribute('role_name', 'test_vehicle')
        vehicle = world.spawn_actor(vehicle_bp, spawn_point)
        print(f"Spawned vehicle with ID: {vehicle.id}")
        
        # Spawn a camera
        camera_bp = bp_lib.find('sensor.camera.rgb')
        camera_bp.set_attribute('role_name', 'test_camera')
        camera_bp.set_attribute('ros_name', 'test_camera')
        camera_bp.set_attribute('image_size_x', '800')
        camera_bp.set_attribute('image_size_y', '600')
        camera_bp.set_attribute('fov', '90')
        camera_bp.set_attribute('sensor_tick', '0.033')  # 30 FPS
        
        camera_transform = carla.Transform(carla.Location(x=2.0, z=1.5))
        camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
        print(f"Spawned camera with ID: {camera.id}")
        print(f"Camera is listening: {camera.is_listening()}")
        
        # Test different approaches
        print("\nTest 1: No callback (default behavior)")
        time.sleep(3)
        print(f"Camera is listening: {camera.is_listening()}")
        
        print("\nTest 2: Add empty callback")
        camera.listen(lambda data: None)
        print(f"Camera is listening: {camera.is_listening()}")
        time.sleep(3)
        
        print("\nTest 3: Stop listening")
        camera.stop()
        print(f"Camera is listening: {camera.is_listening()}")
        time.sleep(3)
        
        print("\nCheck ROS2 topics with: ros2 topic list | grep test_camera")
        print("Keep this script running to maintain the actors...")
        
        # Keep running
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            pass
            
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Cleanup
        if 'camera' in locals():
            camera.destroy()
        if 'vehicle' in locals():
            vehicle.destroy()
        print("Cleaned up")

if __name__ == '__main__':
    main()