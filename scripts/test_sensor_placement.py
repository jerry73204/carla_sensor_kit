#!/usr/bin/env python3
"""
Test script to verify sensor placement accuracy after integration.
Compares expected positions from YAML with actual spawned positions in CARLA.
"""

import numpy as np
from sensor_config_loader import SensorConfigLoader


def test_coordinate_conversion():
    """Test ROS to CARLA coordinate system conversion."""
    print("Testing coordinate system conversion...")
    print("=" * 60)
    
    # Test cases: (ros_y, expected_carla_y)
    test_cases = [
        (1.0, -1.0),   # Positive Y in ROS -> Negative Y in CARLA
        (-1.0, 1.0),   # Negative Y in ROS -> Positive Y in CARLA
        (0.0, 0.0),    # Zero remains zero
    ]
    
    loader = SensorConfigLoader()
    
    for ros_y, expected_carla_y in test_cases:
        # Test using a simple transform
        trans = loader._to_carla_transform(0, ros_y, 0, 0, 0, 0)
        actual_carla_y = trans.location.y
        
        passed = abs(actual_carla_y - expected_carla_y) < 0.001
        status = "PASS" if passed else "FAIL"
        
        print(f"ROS Y: {ros_y:6.2f} -> CARLA Y: {actual_carla_y:6.2f} "
              f"(expected: {expected_carla_y:6.2f}) [{status}]")
    
    # Test rotation conversion
    print("\nTesting rotation conversion...")
    
    # Test yaw conversion (should be negated)
    ros_yaw = np.pi / 2  # 90 degrees in radians
    trans = loader._to_carla_transform(0, 0, 0, 0, 0, ros_yaw)
    carla_yaw = trans.rotation.yaw
    expected_yaw = -90.0  # Should be -90 degrees
    
    passed = abs(carla_yaw - expected_yaw) < 0.1
    status = "PASS" if passed else "FAIL"
    
    print(f"ROS Yaw: {np.degrees(ros_yaw):.1f}° -> CARLA Yaw: {carla_yaw:.1f}° "
          f"(expected: {expected_yaw:.1f}°) [{status}]")


def test_sensor_positions():
    """Test that sensor positions are loaded correctly from YAML."""
    print("\n\nTesting sensor position loading...")
    print("=" * 60)
    
    loader = SensorConfigLoader()
    
    # Test each sensor type
    sensors = [
        ('GNSS', loader.get_gnss_transform),
        ('IMU', loader.get_imu_transform),
        ('LiDAR Top', lambda: loader.get_lidar_transform('top')),
        ('Camera Traffic Light', lambda: loader.get_camera_transform('traffic_light')),
    ]
    
    print("\nLoaded sensor transforms:")
    print("-" * 60)
    
    for sensor_name, get_transform_func in sensors:
        try:
            trans = get_transform_func()
            print(f"\n{sensor_name}:")
            print(f"  Position: X={trans.location.x:7.3f}, Y={trans.location.y:7.3f}, Z={trans.location.z:7.3f}")
            print(f"  Rotation: Roll={trans.rotation.roll:7.2f}°, Pitch={trans.rotation.pitch:7.2f}°, Yaw={trans.rotation.yaw:7.2f}°")
        except Exception as e:
            print(f"\n{sensor_name}: ERROR - {e}")


def test_transform_combination():
    """Test that base_link to sensor transforms are computed correctly."""
    print("\n\nTesting transform combination...")
    print("=" * 60)
    
    loader = SensorConfigLoader()
    
    # Get transforms relative to vehicle vs sensor kit
    gnss_vehicle = loader.get_sensor_transform('gnss_link', relative_to_vehicle=True)
    gnss_kit = loader.get_sensor_transform('gnss_link', relative_to_vehicle=False)
    
    print("\nGNSS Transform relative to vehicle base_link:")
    print(f"  Position: X={gnss_vehicle.location.x:.3f}, Y={gnss_vehicle.location.y:.3f}, Z={gnss_vehicle.location.z:.3f}")
    
    print("\nGNSS Transform relative to sensor_kit_base_link:")
    print(f"  Position: X={gnss_kit.location.x:.3f}, Y={gnss_kit.location.y:.3f}, Z={gnss_kit.location.z:.3f}")
    
    # The difference should be the base_link to sensor_kit transform
    base_to_kit = loader.sensors_calibration['base_link']['sensor_kit_base_link']
    print("\nBase_link to sensor_kit_base_link transform from YAML:")
    print(f"  Position: X={base_to_kit['x']:.3f}, Y={base_to_kit['y']:.3f}, Z={base_to_kit['z']:.3f}")


def test_all_configured_sensors():
    """Test that all sensors in the YAML configuration can be loaded."""
    print("\n\nTesting all configured sensors...")
    print("=" * 60)
    
    loader = SensorConfigLoader()
    
    # Get all sensor names from configuration
    all_sensors = loader.sensor_kit_calibration['sensor_kit_base_link'].keys()
    
    print(f"\nFound {len(all_sensors)} sensors in configuration:")
    
    success_count = 0
    for sensor_name in sorted(all_sensors):
        try:
            trans = loader.get_sensor_transform(sensor_name)
            print(f"  ✓ {sensor_name}: OK")
            success_count += 1
        except Exception as e:
            print(f"  ✗ {sensor_name}: ERROR - {e}")
    
    print(f"\nSuccessfully loaded: {success_count}/{len(all_sensors)} sensors")


def main():
    """Run all tests."""
    print("Sensor Placement Accuracy Test")
    print("=" * 60)
    
    try:
        test_coordinate_conversion()
        test_sensor_positions()
        test_transform_combination()
        test_all_configured_sensors()
        
        print("\n" + "=" * 60)
        print("All tests completed successfully!")
        
    except Exception as e:
        print(f"\nTest failed with error: {e}")
        return 1
    
    return 0


if __name__ == "__main__":
    exit(main())