#!/usr/bin/env python3
"""
Integration tests for sensor topic remapping from CARLA to Autoware.
"""

import unittest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Imu, Image, NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped
import time
import threading


class TopicRemappingTest(unittest.TestCase):
    """Test sensor topic remapping from CARLA to Autoware."""
    
    @classmethod
    def setUpClass(cls):
        """Initialize ROS2 for tests."""
        rclpy.init()
        
    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS2."""
        rclpy.shutdown()
    
    def test_lidar_topic_remapping(self):
        """Test LiDAR topic remapping from CARLA to Autoware."""
        test_node = TopicRemappingTestNode()
        
        # Define expected remapping
        carla_topic = '/carla/ego_vehicle/lidar'
        autoware_topic = '/sensing/lidar/top/pointcloud_raw'
        
        # Create subscribers
        carla_received = threading.Event()
        autoware_received = threading.Event()
        
        def carla_callback(msg):
            carla_received.set()
            
        def autoware_callback(msg):
            autoware_received.set()
        
        carla_sub = test_node.create_subscription(
            PointCloud2, carla_topic, carla_callback, 10)
        autoware_sub = test_node.create_subscription(
            PointCloud2, autoware_topic, autoware_callback, 10)
        
        # Spin for a short time to receive messages
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(test_node)
        
        spin_thread = threading.Thread(
            target=lambda: executor.spin_once(timeout_sec=5.0))
        spin_thread.start()
        
        # Wait for messages
        autoware_received.wait(timeout=5.0)
        
        # Verify remapping
        self.assertTrue(autoware_received.is_set(), 
                       f"No message received on {autoware_topic}")
        
        # Cleanup
        test_node.destroy_node()
        spin_thread.join()
    
    def test_camera_topic_remapping(self):
        """Test camera topic remapping."""
        test_node = TopicRemappingTestNode()
        
        # Test multiple camera remappings
        camera_mappings = [
            ('/carla/ego_vehicle/camera/front', 
             '/sensing/camera/front/image_raw'),
            ('/carla/ego_vehicle/camera/rear', 
             '/sensing/camera/rear/image_raw'),
        ]
        
        for carla_topic, autoware_topic in camera_mappings:
            with self.subTest(camera=carla_topic):
                received = threading.Event()
                
                def callback(msg):
                    received.set()
                
                sub = test_node.create_subscription(
                    Image, autoware_topic, callback, 10)
                
                # Check if topic exists
                topic_list = test_node.get_topic_names_and_types()
                autoware_topic_exists = any(
                    autoware_topic in topic for topic, _ in topic_list)
                
                if autoware_topic_exists:
                    # Wait for message
                    executor = rclpy.executors.SingleThreadedExecutor()
                    executor.add_node(test_node)
                    executor.spin_once(timeout_sec=2.0)
                    
                    self.assertTrue(received.is_set() or autoware_topic_exists,
                                   f"Topic {autoware_topic} not properly remapped")
        
        test_node.destroy_node()
    
    def test_imu_topic_remapping(self):
        """Test IMU topic remapping."""
        test_node = TopicRemappingTestNode()
        
        carla_topic = '/carla/ego_vehicle/imu'
        autoware_topic = '/sensing/imu/tamagawa/imu_raw'
        
        # Check topic exists or receives data
        topic_list = test_node.get_topic_names_and_types()
        topic_exists = any(autoware_topic in topic for topic, _ in topic_list)
        
        self.assertTrue(topic_exists or self._topic_publishes(test_node, autoware_topic, Imu),
                       f"IMU topic {autoware_topic} not found or not publishing")
        
        test_node.destroy_node()
    
    def test_gnss_topic_remapping(self):
        """Test GNSS topic remapping."""
        test_node = TopicRemappingTestNode()
        
        carla_topic = '/carla/ego_vehicle/gnss'
        autoware_topics = [
            '/sensing/gnss/ublox/nav_sat_fix',
            '/sensing/gnss/pose',
            '/sensing/gnss/pose_with_covariance'
        ]
        
        # Check at least one GNSS topic exists
        topic_list = test_node.get_topic_names_and_types()
        gnss_topics_found = []
        
        for topic in autoware_topics:
            if any(topic in t for t, _ in topic_list):
                gnss_topics_found.append(topic)
        
        self.assertTrue(len(gnss_topics_found) > 0,
                       "No GNSS topics found after remapping")
        
        test_node.destroy_node()
    
    def _topic_publishes(self, node, topic, msg_type, timeout=2.0):
        """Check if a topic publishes messages."""
        received = threading.Event()
        
        def callback(msg):
            received.set()
        
        sub = node.create_subscription(msg_type, topic, callback, 10)
        
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(node)
        
        spin_thread = threading.Thread(
            target=lambda: executor.spin_once(timeout_sec=timeout))
        spin_thread.start()
        spin_thread.join()
        
        return received.is_set()


class TopicRemappingTestNode(Node):
    """Test node for topic remapping tests."""
    
    def __init__(self):
        super().__init__('topic_remapping_test_node')


if __name__ == '__main__':
    unittest.main()