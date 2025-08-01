#!/usr/bin/env python3
"""
Unit tests for parameter extraction modules
"""

import unittest
import math
import sys
import os

# Add src to Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from carla_sensor_kit_utils.parameter_extractor import (
    VehicleParameterExtractor,
    SensorParameterExtractor,
    CoordinateTransformer
)


class MockCARLAVector3D:
    """Mock CARLA Vector3D for testing"""
    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z


class MockCARLARotation:
    """Mock CARLA Rotation for testing"""
    def __init__(self, roll=0, pitch=0, yaw=0):
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw


class MockCARLALocation:
    """Mock CARLA Location for testing"""
    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z


class MockCARLATransform:
    """Mock CARLA Transform for testing"""
    def __init__(self, location=None, rotation=None):
        self.location = location or MockCARLALocation()
        self.rotation = rotation or MockCARLARotation()


class MockCARLAWheel:
    """Mock CARLA Wheel for testing"""
    def __init__(self, position, radius=35, max_steer_angle=70):
        self.position = position
        self.radius = radius
        self.max_steer_angle = max_steer_angle


class MockCARLAPhysicsControl:
    """Mock CARLA VehiclePhysicsControl for testing"""
    def __init__(self):
        # Create 4 wheels in typical car configuration
        self.wheels = [
            MockCARLAWheel(MockCARLAVector3D(x=150, y=80, z=0)),   # Front left
            MockCARLAWheel(MockCARLAVector3D(x=150, y=-80, z=0)),  # Front right
            MockCARLAWheel(MockCARLAVector3D(x=-150, y=80, z=0)),  # Rear left
            MockCARLAWheel(MockCARLAVector3D(x=-150, y=-80, z=0))  # Rear right
        ]
        self.mass = 1500.0
        self.drag_coefficient = 0.3
        self.center_of_mass = MockCARLAVector3D(x=0, y=0, z=-50)
        self.max_rpm = 6000.0
        self.moi = 1.0
        self.damping_rate_full_throttle = 0.15
        self.damping_rate_zero_throttle_clutch_engaged = 2.0
        self.damping_rate_zero_throttle_clutch_disengaged = 0.35


class MockCARLABoundingBox:
    """Mock CARLA BoundingBox for testing"""
    def __init__(self, extent):
        self.extent = extent


class MockCARLAVehicle:
    """Mock CARLA Vehicle for testing"""
    def __init__(self):
        self.id = 123
        self.type_id = "vehicle.tesla.model3"
        self.bounding_box = MockCARLABoundingBox(
            MockCARLAVector3D(x=225, y=90, z=75)  # Half extents in cm
        )
        self._physics_control = MockCARLAPhysicsControl()
        
    def get_physics_control(self):
        return self._physics_control
    
    def get_transform(self):
        return MockCARLATransform(
            MockCARLALocation(x=1000, y=2000, z=100),
            MockCARLARotation(roll=0, pitch=0, yaw=45)
        )
    
    def get_velocity(self):
        return MockCARLAVector3D(x=1000, y=0, z=0)  # 10 m/s forward


class MockCARLASensor:
    """Mock CARLA Sensor for testing"""
    def __init__(self, type_id, sensor_id, parent_id, attributes, transform):
        self.type_id = type_id
        self.id = sensor_id
        self.parent = MockCARLAVehicle() if parent_id else None
        if self.parent:
            self.parent.id = parent_id
        self.attributes = attributes
        self._transform = transform
        
    def get_transform(self):
        return self._transform


class MockCARLAWorld:
    """Mock CARLA World for testing"""
    def __init__(self, sensors):
        self.sensors = sensors
        
    def get_actors(self):
        return MockActorList(self.sensors)


class MockActorList:
    """Mock actor list for testing"""
    def __init__(self, actors):
        self.actors = actors
        
    def __iter__(self):
        return iter(self.actors)


class TestVehicleParameterExtractor(unittest.TestCase):
    """Test VehicleParameterExtractor class"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.vehicle = MockCARLAVehicle()
        self.extractor = VehicleParameterExtractor(self.vehicle)
    
    def test_extract_wheel_parameters(self):
        """Test wheel parameter extraction"""
        params = self.extractor.extract_wheel_parameters()
        
        # Check wheel radius (35 cm -> 0.35 m)
        self.assertAlmostEqual(params['wheel_radius'], 0.35, places=3)
        
        # Check wheel base (300 cm -> 3.0 m)
        self.assertAlmostEqual(params['wheel_base'], 3.0, places=3)
        
        # Check wheel tread (160 cm -> 1.6 m)
        self.assertAlmostEqual(params['wheel_tread'], 1.6, places=3)
        
        # Check max steer angle (70 deg -> 1.22 rad)
        self.assertAlmostEqual(params['max_steer_angle'], math.radians(70), places=3)
    
    def test_extract_vehicle_dimensions(self):
        """Test vehicle dimension extraction"""
        params = self.extractor.extract_vehicle_dimensions()
        
        # Check vehicle dimensions (half extents * 2, cm to m)
        self.assertAlmostEqual(params['vehicle_length'], 4.5, places=3)
        self.assertAlmostEqual(params['vehicle_width'], 1.8, places=3)
        self.assertAlmostEqual(params['vehicle_height'], 1.5, places=3)
        
        # Check overhangs
        self.assertGreater(params['front_overhang'], 0)
        self.assertGreater(params['rear_overhang'], 0)
        self.assertGreater(params['left_overhang'], 0)
        self.assertGreater(params['right_overhang'], 0)
    
    def test_extract_physics_parameters(self):
        """Test physics parameter extraction"""
        params = self.extractor.extract_physics_parameters()
        
        self.assertEqual(params['mass'], 1500.0)
        self.assertEqual(params['drag_coefficient'], 0.3)
        self.assertEqual(params['max_rpm'], 6000.0)
        
        # Check center of mass conversion (cm to m)
        self.assertAlmostEqual(params['center_of_mass']['x'], 0.0, places=3)
        self.assertAlmostEqual(params['center_of_mass']['y'], 0.0, places=3)
        self.assertAlmostEqual(params['center_of_mass']['z'], -0.5, places=3)
    
    def test_extract_all_parameters(self):
        """Test extraction of all parameters"""
        params = self.extractor.extract_all_parameters()
        
        # Check structure
        self.assertIn('vehicle_info', params)
        self.assertIn('simulator_model', params)
        self.assertIn('physics', params)
        self.assertIn('dimensions', params)
        self.assertIn('wheels', params)
        
        # Check vehicle_info has required fields
        vehicle_info = params['vehicle_info']
        required_fields = [
            'wheel_radius', 'wheel_width', 'wheel_base', 'wheel_tread',
            'front_overhang', 'rear_overhang', 'left_overhang', 'right_overhang',
            'vehicle_height', 'max_steer_angle'
        ]
        for field in required_fields:
            self.assertIn(field, vehicle_info)


class TestSensorParameterExtractor(unittest.TestCase):
    """Test SensorParameterExtractor class"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.vehicle = MockCARLAVehicle()
        
        # Create mock sensors
        self.sensors = [
            MockCARLASensor(
                'sensor.camera.rgb',
                201,
                123,  # Parent vehicle ID
                {
                    'role_name': 'camera_front',
                    'image_size_x': '1920',
                    'image_size_y': '1080',
                    'fov': '90'
                },
                MockCARLATransform(
                    MockCARLALocation(x=1150, y=2000, z=200),  # 150cm forward, 100cm up
                    MockCARLARotation(roll=0, pitch=0, yaw=45)
                )
            ),
            MockCARLASensor(
                'sensor.lidar.ray_cast',
                202,
                123,
                {
                    'role_name': 'lidar_top',
                    'channels': '64',
                    'range': '100',
                    'rotation_frequency': '20'
                },
                MockCARLATransform(
                    MockCARLALocation(x=1000, y=2000, z=300),  # 200cm up
                    MockCARLARotation(roll=0, pitch=0, yaw=45)
                )
            )
        ]
        
        self.world = MockCARLAWorld(self.sensors)
        self.extractor = SensorParameterExtractor(self.world, self.vehicle)
    
    def test_extract_camera_parameters(self):
        """Test camera parameter extraction"""
        camera = self.sensors[0]
        params = self.extractor.extract_camera_parameters(camera)
        
        self.assertEqual(params['sensor_type'], 'camera')
        self.assertEqual(params['image_width'], 1920)
        self.assertEqual(params['image_height'], 1080)
        self.assertEqual(params['fov'], 90.0)
    
    def test_extract_lidar_parameters(self):
        """Test LiDAR parameter extraction"""
        lidar = self.sensors[1]
        params = self.extractor.extract_lidar_parameters(lidar)
        
        self.assertEqual(params['sensor_type'], 'lidar')
        self.assertEqual(params['channels'], 64)
        self.assertEqual(params['range'], 100.0)
        self.assertEqual(params['rotation_frequency'], 20.0)
    
    def test_extract_sensor_transform(self):
        """Test sensor transform extraction"""
        camera = self.sensors[0]
        transform = self.extractor.extract_sensor_transform(camera)
        
        # Check that transform is relative to vehicle
        # Camera is 150cm forward of vehicle
        self.assertAlmostEqual(transform['x'], 1.5, places=2)
        self.assertAlmostEqual(transform['z'], 1.0, places=2)
        
        # Check coordinate system conversion
        self.assertIsInstance(transform['roll'], float)
        self.assertIsInstance(transform['pitch'], float)
        self.assertIsInstance(transform['yaw'], float)
    
    def test_extract_all_sensor_parameters(self):
        """Test extraction of all sensor parameters"""
        params = self.extractor.extract_all_sensor_parameters()
        
        # Should have 2 sensors
        self.assertEqual(len(params), 2)
        
        # Check camera_front
        self.assertIn('camera_front', params)
        camera_params = params['camera_front']
        self.assertEqual(camera_params['sensor_type'], 'camera')
        self.assertEqual(camera_params['image_width'], 1920)
        
        # Check lidar_top
        self.assertIn('lidar_top', params)
        lidar_params = params['lidar_top']
        self.assertEqual(lidar_params['sensor_type'], 'lidar')
        self.assertEqual(lidar_params['channels'], 64)


class TestCoordinateTransformer(unittest.TestCase):
    """Test CoordinateTransformer class"""
    
    def test_carla_to_ros_position(self):
        """Test CARLA to ROS position conversion"""
        carla_pos = MockCARLAVector3D(x=100, y=200, z=300)  # cm
        ros_pos = CoordinateTransformer.carla_to_ros_position(carla_pos)
        
        # Check conversion (cm to m, flip Y)
        self.assertAlmostEqual(ros_pos['x'], 1.0, places=3)
        self.assertAlmostEqual(ros_pos['y'], -2.0, places=3)
        self.assertAlmostEqual(ros_pos['z'], 3.0, places=3)
    
    def test_carla_to_ros_rotation(self):
        """Test CARLA to ROS rotation conversion"""
        carla_rot = MockCARLARotation(roll=30, pitch=45, yaw=60)  # degrees
        ros_rot = CoordinateTransformer.carla_to_ros_rotation(carla_rot)
        
        # Check conversion (deg to rad, flip pitch and yaw)
        self.assertAlmostEqual(ros_rot['roll'], math.radians(30), places=3)
        self.assertAlmostEqual(ros_rot['pitch'], math.radians(-45), places=3)
        self.assertAlmostEqual(ros_rot['yaw'], math.radians(-60), places=3)
    
    def test_calculate_quaternion(self):
        """Test quaternion calculation"""
        # Test identity rotation
        quat = CoordinateTransformer.calculate_quaternion(0, 0, 0)
        self.assertAlmostEqual(quat['w'], 1.0, places=3)
        self.assertAlmostEqual(quat['x'], 0.0, places=3)
        self.assertAlmostEqual(quat['y'], 0.0, places=3)
        self.assertAlmostEqual(quat['z'], 0.0, places=3)
        
        # Test 90 degree yaw rotation
        quat = CoordinateTransformer.calculate_quaternion(0, 0, math.pi/2)
        self.assertAlmostEqual(quat['w'], 0.707, places=3)
        self.assertAlmostEqual(quat['z'], 0.707, places=3)
    
    def test_transform_velocity(self):
        """Test velocity transformation"""
        # Vehicle moving forward at 10 m/s
        velocity = MockCARLAVector3D(x=1000, y=0, z=0)  # cm/s
        yaw = 0  # No rotation
        
        vx, vy, vz = CoordinateTransformer.transform_velocity(velocity, yaw)
        self.assertAlmostEqual(vx, 10.0, places=3)
        self.assertAlmostEqual(vy, 0.0, places=3)
        self.assertAlmostEqual(vz, 0.0, places=3)
        
        # Vehicle rotated 90 degrees
        yaw = math.pi / 2
        vx, vy, vz = CoordinateTransformer.transform_velocity(velocity, yaw)
        self.assertAlmostEqual(vx, 0.0, places=2)
        self.assertAlmostEqual(vy, -10.0, places=2)  # Flipped due to ROS convention


if __name__ == '__main__':
    unittest.main()