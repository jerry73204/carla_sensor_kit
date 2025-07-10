#!/usr/bin/env python3
"""
Sensor configuration loader for CARLA-Autoware integration.
Loads sensor poses from YAML files and provides them to the CARLA spawner.
"""

import yaml
import carla
from typing import Dict, Optional
import numpy as np
from pathlib import Path


class SensorConfigLoader:
    """Loads sensor configurations from Autoware sensor kit YAML files."""
    
    def __init__(self, sensor_kit_path: Optional[str] = None):
        """
        Initialize the sensor configuration loader.
        
        Args:
            sensor_kit_path: Path to sensor kit description package.
                           If None, will try to find it automatically.
        """
        if sensor_kit_path is None:
            # Try to find the sensor kit package
            possible_paths = [
                Path(__file__).parent.parent / "carla_sensor_kit_description",
                Path.home() / "repos" / "carla_sensor_kit_launch" / "carla_sensor_kit_description",
                Path("/home/aeon/repos/carla_sensor_kit_launch/carla_sensor_kit_description")
            ]
            
            for path in possible_paths:
                if path.exists():
                    sensor_kit_path = str(path)
                    break
            
            if sensor_kit_path is None:
                raise RuntimeError("Could not find carla_sensor_kit_description package")
        
        self.sensor_kit_path = Path(sensor_kit_path)
        self.config_path = self.sensor_kit_path / "config"
        
        # Load configuration files
        self.sensors_calibration = self._load_yaml("sensors_calibration.yaml")
        self.sensor_kit_calibration = self._load_yaml("sensor_kit_calibration.yaml")
    
    def _load_yaml(self, filename: str) -> dict:
        """Load a YAML file from the config directory."""
        filepath = self.config_path / filename
        if not filepath.exists():
            raise FileNotFoundError(f"Configuration file not found: {filepath}")
        
        with open(filepath, 'r') as f:
            return yaml.safe_load(f)
    
    def _to_carla_transform(self, x: float, y: float, z: float, 
                           roll: float, pitch: float, yaw: float) -> carla.Transform:
        """
        Convert Autoware coordinate system to CARLA Transform.
        
        Autoware uses ROS REP-103 (x-forward, y-left, z-up)
        CARLA uses Unreal Engine coords (x-forward, y-right, z-up)
        
        Args:
            x, y, z: Position in meters
            roll, pitch, yaw: Rotation in radians
            
        Returns:
            carla.Transform object
        """
        # Convert y-axis (flip sign for left-hand to right-hand conversion)
        carla_y = -y
        
        # Convert rotations from radians to degrees
        # Also adjust for coordinate system differences
        carla_roll = np.degrees(roll)
        carla_pitch = np.degrees(pitch)
        carla_yaw = np.degrees(-yaw)  # Flip yaw for coordinate system
        
        return carla.Transform(
            carla.Location(x=x, y=carla_y, z=z),
            carla.Rotation(pitch=carla_pitch, yaw=carla_yaw, roll=carla_roll)
        )
    
    def get_sensor_transform(self, sensor_name: str, 
                           relative_to_vehicle: bool = True) -> carla.Transform:
        """
        Get sensor transform in CARLA coordinate system.
        
        Args:
            sensor_name: Name of the sensor (e.g., 'velodyne_top_base_link', 'gnss_link', 
                        'tamagawa/imu_link', 'traffic_light_left_camera/camera_link')
            relative_to_vehicle: If True, return position relative to vehicle base_link.
                               If False, return position relative to sensor_kit_base_link.
                               
        Returns:
            carla.Transform object
        """
        # Get sensor transform relative to sensor kit
        kit_to_sensor = self.sensor_kit_calibration['sensor_kit_base_link'].get(sensor_name)
        if kit_to_sensor is None:
            raise KeyError(f"Sensor '{sensor_name}' not found in configuration")
        
        if not relative_to_vehicle:
            # Return transform relative to sensor kit
            return self._to_carla_transform(**kit_to_sensor)
        
        # Compute full transform from base_link to sensor
        base_to_kit = self.sensors_calibration['base_link']['sensor_kit_base_link']
        
        # Combine transforms (simplified - assumes small angles)
        # For more accurate results, use transformation matrices
        combined = {
            'x': base_to_kit['x'] + kit_to_sensor['x'],
            'y': base_to_kit['y'] + kit_to_sensor['y'],
            'z': base_to_kit['z'] + kit_to_sensor['z'],
            'roll': base_to_kit['roll'] + kit_to_sensor['roll'],
            'pitch': base_to_kit['pitch'] + kit_to_sensor['pitch'],
            'yaw': base_to_kit['yaw'] + kit_to_sensor['yaw']
        }
        
        return self._to_carla_transform(**combined)
    
    def get_gnss_transform(self) -> carla.Transform:
        """Get GNSS sensor transform."""
        return self.get_sensor_transform('gnss_link')
    
    def get_imu_transform(self) -> carla.Transform:
        """Get IMU sensor transform."""
        return self.get_sensor_transform('tamagawa/imu_link')
    
    def get_lidar_transform(self, lidar_type: str = 'top') -> carla.Transform:
        """Get LiDAR sensor transform by type (top, left, right)."""
        name_map = {
            'top': 'velodyne_top_base_link',
            'left': 'velodyne_left_base_link',
            'right': 'velodyne_right_base_link'
        }
        sensor_name = name_map.get(lidar_type)
        if sensor_name is None:
            raise KeyError(f"Unknown LiDAR type: {lidar_type}")
        return self.get_sensor_transform(sensor_name)
    
    def get_camera_transform(self, camera_type: str = 'traffic_light_left') -> carla.Transform:
        """Get camera sensor transform by type."""
        # Map common camera types to their configuration names
        name_map = {
            'traffic_light': 'traffic_light_left_camera/camera_link',
            'traffic_light_left': 'traffic_light_left_camera/camera_link',
            'traffic_light_right': 'traffic_light_right_camera/camera_link',
            'front': 'camera0/camera_link',
            'rear': 'camera4/camera_link'
        }
        sensor_name = name_map.get(camera_type)
        if sensor_name is None:
            # Try direct name
            sensor_name = camera_type
        return self.get_sensor_transform(sensor_name)
    


# Example usage for direct integration with CARLA spawner
def get_sensor_transforms():
    """
    Get all sensor transforms for use in CARLA spawner.
    
    Returns:
        Dictionary with sensor names as keys and carla.Transform objects as values
    """
    loader = SensorConfigLoader()
    
    return {
        'gnss': loader.get_gnss_transform(),
        'imu': loader.get_imu_transform(),
        'lidar_top': loader.get_lidar_transform('top'),
        'camera_traffic_light': loader.get_camera_transform('traffic_light')
    }


if __name__ == "__main__":
    # Test the loader
    print("Testing sensor configuration loader...\n")
    
    try:
        loader = SensorConfigLoader()
        print("Successfully loaded sensor configurations from:")
        print(f"  - {loader.config_path}/sensors_calibration.yaml")
        print(f"  - {loader.config_path}/sensor_kit_calibration.yaml")
        
        print("\nExample transforms:")
        gnss = loader.get_gnss_transform()
        print(f"\nGNSS: Location({gnss.location.x:.2f}, {gnss.location.y:.2f}, {gnss.location.z:.2f}), "
              f"Rotation({gnss.rotation.roll:.1f}°, {gnss.rotation.pitch:.1f}°, {gnss.rotation.yaw:.1f}°)")
        
        imu = loader.get_imu_transform()
        print(f"\nIMU: Location({imu.location.x:.2f}, {imu.location.y:.2f}, {imu.location.z:.2f}), "
              f"Rotation({imu.rotation.roll:.1f}°, {imu.rotation.pitch:.1f}°, {imu.rotation.yaw:.1f}°)")
        
        lidar = loader.get_lidar_transform('top')
        print(f"\nTop LiDAR: Location({lidar.location.x:.2f}, {lidar.location.y:.2f}, {lidar.location.z:.2f}), "
              f"Rotation({lidar.rotation.roll:.1f}°, {lidar.rotation.pitch:.1f}°, {lidar.rotation.yaw:.1f}°)")
        
    except Exception as e:
        print(f"Error: {e}")
        exit(1)