#!/usr/bin/env python3
"""
Unit tests for configuration loading functionality.
"""

import unittest
import sys
import yaml
import tempfile
from pathlib import Path
import numpy as np

# Add scripts directory to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent / "scripts"))
from sensor_config_loader import SensorConfigLoader


class ConfigLoadingTest(unittest.TestCase):
    """Test configuration file loading and parsing."""
    
    def test_yaml_loading(self):
        """Test loading YAML configuration files."""
        loader = SensorConfigLoader()
        
        # Verify configurations are loaded
        self.assertIsNotNone(loader.sensors_calibration)
        self.assertIsNotNone(loader.sensor_kit_calibration)
        
        # Check expected structure
        self.assertIn('base_link', loader.sensors_calibration)
        self.assertIn('sensor_kit_base_link', loader.sensor_kit_calibration)
    
    def test_sensor_name_lookup(self):
        """Test sensor configuration lookup by name."""
        loader = SensorConfigLoader()
        
        # Test valid sensor names
        valid_sensors = ['gnss_link', 'tamagawa/imu_link', 'velodyne_top_base_link']
        
        for sensor in valid_sensors:
            with self.subTest(sensor=sensor):
                transform = loader.get_sensor_transform(sensor)
                self.assertIsNotNone(transform)
                self.assertIsNotNone(transform.location)
                self.assertIsNotNone(transform.rotation)
    
    def test_invalid_sensor_lookup(self):
        """Test handling of invalid sensor names."""
        loader = SensorConfigLoader()
        
        with self.assertRaises(KeyError):
            loader.get_sensor_transform('invalid_sensor_name')
    
    def test_coordinate_conversion(self):
        """Test ROS to CARLA coordinate system conversion."""
        loader = SensorConfigLoader()
        
        # Test coordinate conversion
        test_cases = [
            # (x, y, z, roll, pitch, yaw) in ROS
            (1.0, 2.0, 3.0, 0.0, 0.0, 0.0),
            (0.0, 1.0, 0.0, 0.0, 0.0, np.pi/2),
            (-1.0, -2.0, 1.5, np.pi/4, 0.0, -np.pi/2)
        ]
        
        for x, y, z, roll, pitch, yaw in test_cases:
            with self.subTest(coords=(x, y, z)):
                transform = loader._to_carla_transform(x, y, z, roll, pitch, yaw)
                
                # Check coordinate conversion rules
                self.assertEqual(transform.location.x, x)
                self.assertEqual(transform.location.y, -y)  # Y axis flipped
                self.assertEqual(transform.location.z, z)
                
                # Check rotation conversion
                self.assertAlmostEqual(transform.rotation.roll, np.degrees(roll), places=2)
                self.assertAlmostEqual(transform.rotation.pitch, np.degrees(pitch), places=2)
                self.assertAlmostEqual(transform.rotation.yaw, np.degrees(-yaw), places=2)
    
    def test_transform_combination(self):
        """Test combining base_link and sensor_kit transforms."""
        loader = SensorConfigLoader()
        
        # Test relative transforms
        gnss_vehicle = loader.get_sensor_transform('gnss_link', relative_to_vehicle=True)
        gnss_kit = loader.get_sensor_transform('gnss_link', relative_to_vehicle=False)
        
        # Transforms should be different
        self.assertNotEqual(gnss_vehicle.location.x, gnss_kit.location.x)
        self.assertNotEqual(gnss_vehicle.location.z, gnss_kit.location.z)
    
    def test_helper_methods(self):
        """Test convenience helper methods."""
        loader = SensorConfigLoader()
        
        # Test GNSS helper
        gnss_transform = loader.get_gnss_transform()
        self.assertIsNotNone(gnss_transform)
        
        # Test IMU helper
        imu_transform = loader.get_imu_transform()
        self.assertIsNotNone(imu_transform)
        
        # Test LiDAR helper
        lidar_transform = loader.get_lidar_transform('top')
        self.assertIsNotNone(lidar_transform)
        
        # Test invalid LiDAR type
        with self.assertRaises(KeyError):
            loader.get_lidar_transform('invalid_type')
        
        # Test camera helper
        camera_transform = loader.get_camera_transform('traffic_light')
        self.assertIsNotNone(camera_transform)
    
    def test_config_file_missing(self):
        """Test handling of missing configuration files."""
        with tempfile.TemporaryDirectory() as tmpdir:
            # Try to load from empty directory
            with self.assertRaises(FileNotFoundError):
                loader = SensorConfigLoader(sensor_kit_path=tmpdir)
    
    def test_all_sensors_loadable(self):
        """Test that all sensors in config can be loaded."""
        loader = SensorConfigLoader()
        
        # Get all configured sensors
        all_sensors = loader.sensor_kit_calibration['sensor_kit_base_link'].keys()
        
        failed_sensors = []
        for sensor in all_sensors:
            try:
                transform = loader.get_sensor_transform(sensor)
                self.assertIsNotNone(transform)
            except Exception as e:
                failed_sensors.append((sensor, str(e)))
        
        self.assertEqual(len(failed_sensors), 0,
                        f"Failed to load sensors: {failed_sensors}")


if __name__ == '__main__':
    unittest.main()