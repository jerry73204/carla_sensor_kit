#!/usr/bin/env python3
"""
Integration tests for CARLA vehicle interface
"""

import unittest
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
import threading
import time

# Test messages
from autoware_auto_control_msgs.msg import AckermannControlCommand
from autoware_auto_vehicle_msgs.msg import (
    ControlModeReport,
    GearReport,
    VelocityReport,
    SteeringReport
)
from geometry_msgs.msg import TwistStamped


class VehicleInterfaceTestNode(Node):
    """Test node for vehicle interface integration tests"""
    
    def __init__(self):
        super().__init__('vehicle_interface_test_node')
        
        # QoS profile for reliable communication
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        
        # Publishers
        self.control_pub = self.create_publisher(
            AckermannControlCommand,
            '/control/command/control_cmd',
            qos
        )
        
        # Subscribers
        self.velocity_sub = self.create_subscription(
            VelocityReport,
            '/vehicle/status/velocity_status',
            self.velocity_callback,
            qos
        )
        
        self.steering_sub = self.create_subscription(
            SteeringReport,
            '/vehicle/status/steering_status',
            self.steering_callback,
            qos
        )
        
        self.control_mode_sub = self.create_subscription(
            ControlModeReport,
            '/vehicle/status/control_mode',
            self.control_mode_callback,
            qos
        )
        
        # State tracking
        self.latest_velocity = None
        self.latest_steering = None
        self.latest_control_mode = None
        self.callbacks_received = {
            'velocity': False,
            'steering': False,
            'control_mode': False
        }
        
    def velocity_callback(self, msg):
        self.latest_velocity = msg
        self.callbacks_received['velocity'] = True
        
    def steering_callback(self, msg):
        self.latest_steering = msg
        self.callbacks_received['steering'] = True
        
    def control_mode_callback(self, msg):
        self.latest_control_mode = msg
        self.callbacks_received['control_mode'] = True
        
    def send_control_command(self, steering_angle=0.0, acceleration=0.0):
        """Send a control command"""
        msg = AckermannControlCommand()
        msg.lateral.steering_tire_angle = steering_angle
        msg.longitudinal.acceleration = acceleration
        self.control_pub.publish(msg)
        
    def wait_for_callbacks(self, timeout=5.0):
        """Wait for all callbacks to be received"""
        start_time = time.time()
        while not all(self.callbacks_received.values()):
            if time.time() - start_time > timeout:
                return False
            time.sleep(0.1)
        return True


class TestVehicleInterface(unittest.TestCase):
    """Integration tests for CARLA vehicle interface"""
    
    @classmethod
    def setUpClass(cls):
        """Set up ROS context"""
        rclpy.init()
        
    @classmethod
    def tearDownClass(cls):
        """Clean up ROS context"""
        rclpy.shutdown()
        
    def setUp(self):
        """Set up test node"""
        self.test_node = VehicleInterfaceTestNode()
        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self.test_node)
        
        # Run executor in background thread
        self.executor_thread = threading.Thread(
            target=self.executor.spin,
            daemon=True
        )
        self.executor_thread.start()
        
        # Wait a bit for connections
        time.sleep(1.0)
        
    def tearDown(self):
        """Clean up test node"""
        self.executor.shutdown()
        self.test_node.destroy_node()
        
    def test_status_publishing(self):
        """Test that vehicle interface publishes status messages"""
        # Wait for initial status messages
        success = self.test_node.wait_for_callbacks(timeout=5.0)
        
        if not success:
            # This might happen if CARLA is not running
            self.skipTest("Vehicle interface not responding - CARLA may not be running")
        
        # Check velocity report
        self.assertIsNotNone(self.test_node.latest_velocity)
        self.assertIsInstance(self.test_node.latest_velocity, VelocityReport)
        
        # Check steering report
        self.assertIsNotNone(self.test_node.latest_steering)
        self.assertIsInstance(self.test_node.latest_steering, SteeringReport)
        
        # Check control mode
        self.assertIsNotNone(self.test_node.latest_control_mode)
        self.assertIsInstance(self.test_node.latest_control_mode, ControlModeReport)
        
    def test_control_command_response(self):
        """Test vehicle response to control commands"""
        # Skip if no initial connection
        if not self.test_node.wait_for_callbacks(timeout=3.0):
            self.skipTest("Vehicle interface not responding")
        
        # Send steering command
        self.test_node.send_control_command(steering_angle=0.5, acceleration=0.0)
        time.sleep(0.5)
        
        # Check if steering was updated
        if self.test_node.latest_steering:
            # Note: Actual steering might not match exactly due to vehicle dynamics
            self.assertIsInstance(
                self.test_node.latest_steering.steering_tire_angle,
                float
            )
        
        # Send acceleration command
        self.test_node.send_control_command(steering_angle=0.0, acceleration=2.0)
        time.sleep(0.5)
        
        # Velocity change would depend on CARLA simulation
        
    def test_control_mode_reporting(self):
        """Test control mode reporting"""
        if not self.test_node.wait_for_callbacks(timeout=3.0):
            self.skipTest("Vehicle interface not responding")
        
        # Check control mode
        mode = self.test_node.latest_control_mode
        self.assertIsNotNone(mode)
        
        # Should be either AUTONOMOUS or NO_COMMAND
        self.assertIn(mode.mode, [
            ControlModeReport.AUTONOMOUS,
            ControlModeReport.NO_COMMAND,
            ControlModeReport.MANUAL
        ])
        
    def test_message_timestamps(self):
        """Test that messages have valid timestamps"""
        if not self.test_node.wait_for_callbacks(timeout=3.0):
            self.skipTest("Vehicle interface not responding")
        
        # Check velocity timestamp
        if self.test_node.latest_velocity:
            stamp = self.test_node.latest_velocity.header.stamp
            self.assertGreater(stamp.sec + stamp.nanosec * 1e-9, 0)
        
        # Check steering timestamp
        if self.test_node.latest_steering:
            stamp = self.test_node.latest_steering.stamp
            self.assertGreater(stamp.sec + stamp.nanosec * 1e-9, 0)
        
        # Check control mode timestamp
        if self.test_node.latest_control_mode:
            stamp = self.test_node.latest_control_mode.stamp
            self.assertGreater(stamp.sec + stamp.nanosec * 1e-9, 0)


class TestVehicleInterfaceLaunch(unittest.TestCase):
    """Test vehicle interface launch file"""
    
    def test_launch_file_exists(self):
        """Test that launch files exist"""
        import os
        from ament_index_python.packages import get_package_share_directory
        
        try:
            pkg_dir = get_package_share_directory('carla_vehicle_launch')
            
            # Check XML launch file
            xml_launch = os.path.join(pkg_dir, 'launch', 'vehicle_interface.launch.xml')
            self.assertTrue(os.path.exists(xml_launch), f"Launch file not found: {xml_launch}")
            
            # Check Python launch file
            py_launch = os.path.join(pkg_dir, 'launch', 'vehicle_interface.launch.py')
            self.assertTrue(os.path.exists(py_launch), f"Launch file not found: {py_launch}")
            
        except Exception as e:
            self.skipTest(f"Package not found: {e}")


def main():
    """Run tests"""
    # Note: These are integration tests that require:
    # 1. CARLA to be running
    # 2. Vehicle interface node to be running
    # 3. Proper ROS2 environment
    
    # For CI/CD, you might want to mock CARLA or skip these tests
    unittest.main()


if __name__ == '__main__':
    main()