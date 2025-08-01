#!/usr/bin/env python3
"""
CARLA to ROS2 bridge for sensor data.
Connects to CARLA and publishes sensor data to ROS2 topics.
"""

import sys
import time
import argparse
import numpy as np
from typing import Dict, List, Optional
import weakref
import queue

try:
    import carla
except ImportError:
    print("Error: CARLA Python API not found")
    print("Please add CARLA PythonAPI to PYTHONPATH")
    sys.exit(1)

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# ROS2 message types
from sensor_msgs.msg import PointCloud2, PointField, Image, Imu, NavSatFix
from geometry_msgs.msg import TwistWithCovarianceStamped, TwistStamped
from std_msgs.msg import Header
from builtin_interfaces.msg import Time


class CarlaSensorBridge(Node):
    """Bridge between CARLA sensors and ROS2 topics."""

    def __init__(self, host: str = None, port: int = None, role_name: str = None):
        super().__init__("carla_sensor_bridge")

        # Declare parameters
        self.declare_parameter('carla_host', 'localhost')
        self.declare_parameter('carla_port', 2000)
        self.declare_parameter('vehicle_role_name', 'ego_vehicle')
        self.declare_parameter('use_sim_time', False)
        
        # Get parameters (command line args override ROS params if provided)
        self.host = host or self.get_parameter('carla_host').value
        self.port = port or self.get_parameter('carla_port').value
        self.role_name = role_name or self.get_parameter('vehicle_role_name').value
        
        self.client = None
        self.world = None
        self.vehicle = None
        self.sensors = []
        self.sensor_queues = {}

        # Create publishers
        self.publishers = self._create_publishers()

        # Connect to CARLA
        self._connect_to_carla()

        # Find vehicle and attach sensor callbacks
        self._setup_sensors()

        # Create timer for processing sensor data
        self.timer = self.create_timer(0.01, self._process_sensor_data)

        self.get_logger().info(f"CARLA ROS2 bridge started for vehicle '{self.role_name}'")

    def _create_publishers(self) -> Dict:
        """Create ROS2 publishers for sensor data."""
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1,
        )

        return {
            "lidar": self.create_publisher(
                PointCloud2, "/sensing/lidar/top/pointcloud_raw", qos
            ),
            "camera_front": self.create_publisher(
                Image, "/sensing/camera/camera_front/image_raw", qos
            ),
            "camera_rear": self.create_publisher(
                Image, "/sensing/camera/camera_rear/image_raw", qos
            ),
            "imu": self.create_publisher(Imu, "/sensing/imu/imu_data", qos),
            "gnss": self.create_publisher(NavSatFix, "/sensing/gnss/gnss_data", qos),
            "velocity": self.create_publisher(
                TwistWithCovarianceStamped, "/vehicle/status/velocity_status", qos
            ),
        }

    def _connect_to_carla(self):
        """Connect to CARLA server."""
        try:
            self.client = carla.Client(self.host, self.port)
            self.client.set_timeout(10.0)
            self.world = self.client.get_world()
            self.get_logger().info(f"Connected to CARLA at {self.host}:{self.port}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to CARLA: {e}")
            raise

    def _setup_sensors(self):
        """Find vehicle and set up sensor callbacks."""
        # Find vehicle by role name
        actors = self.world.get_actors()
        vehicles = actors.filter("vehicle.*")

        for vehicle in vehicles:
            if vehicle.attributes.get("role_name") == self.role_name:
                self.vehicle = vehicle
                break

        if self.vehicle is None:
            self.get_logger().error(
                f"No vehicle found with role_name '{self.role_name}'"
            )
            return

        self.get_logger().info(f"Found vehicle: {self.vehicle.type_id}")

        # Find all sensors attached to the vehicle
        for actor in actors:
            if (
                hasattr(actor, "parent")
                and actor.parent
                and actor.parent.id == self.vehicle.id
            ):
                if "sensor" in actor.type_id:
                    self._attach_sensor_callback(actor)

    def _attach_sensor_callback(self, sensor):
        """Attach callback to sensor based on its type."""
        sensor_name = sensor.attributes.get("role_name", f"sensor_{sensor.id}")
        self.sensor_queues[sensor_name] = queue.Queue(maxsize=10)

        weak_self = weakref.ref(self)

        if "lidar" in sensor.type_id:
            sensor.listen(
                lambda data: CarlaSensorBridge._on_lidar_data(
                    weak_self, sensor_name, data
                )
            )
            self.get_logger().info(f"Attached callback to LiDAR sensor: {sensor_name}")
        elif "camera.rgb" in sensor.type_id:
            sensor.listen(
                lambda data: CarlaSensorBridge._on_camera_data(
                    weak_self, sensor_name, data
                )
            )
            self.get_logger().info(f"Attached callback to camera sensor: {sensor_name}")
        elif "imu" in sensor.type_id:
            sensor.listen(
                lambda data: CarlaSensorBridge._on_imu_data(
                    weak_self, sensor_name, data
                )
            )
            self.get_logger().info(f"Attached callback to IMU sensor: {sensor_name}")
        elif "gnss" in sensor.type_id:
            sensor.listen(
                lambda data: CarlaSensorBridge._on_gnss_data(
                    weak_self, sensor_name, data
                )
            )
            self.get_logger().info(f"Attached callback to GNSS sensor: {sensor_name}")

        self.sensors.append(sensor)

    @staticmethod
    def _on_lidar_data(weak_self, sensor_name: str, data):
        """Callback for LiDAR data."""
        self = weak_self()
        if not self:
            return

        try:
            self.sensor_queues[sensor_name].put_nowait(("lidar", data))
        except queue.Full:
            pass

    @staticmethod
    def _on_camera_data(weak_self, sensor_name: str, data):
        """Callback for camera data."""
        self = weak_self()
        if not self:
            return

        try:
            self.sensor_queues[sensor_name].put_nowait((sensor_name, data))
        except queue.Full:
            pass

    @staticmethod
    def _on_imu_data(weak_self, sensor_name: str, data):
        """Callback for IMU data."""
        self = weak_self()
        if not self:
            return

        try:
            self.sensor_queues[sensor_name].put_nowait(("imu", data))
        except queue.Full:
            pass

    @staticmethod
    def _on_gnss_data(weak_self, sensor_name: str, data):
        """Callback for GNSS data."""
        self = weak_self()
        if not self:
            return

        try:
            self.sensor_queues[sensor_name].put_nowait(("gnss", data))
        except queue.Full:
            pass

    def _process_sensor_data(self):
        """Process queued sensor data and publish to ROS2."""
        # Process all sensor queues
        for sensor_name, sensor_queue in self.sensor_queues.items():
            while not sensor_queue.empty():
                try:
                    sensor_type, data = sensor_queue.get_nowait()

                    if sensor_type == "lidar":
                        self._publish_lidar(data)
                    elif "camera_front" in sensor_name:
                        self._publish_camera(data, "camera_front")
                    elif "camera_rear" in sensor_name:
                        self._publish_camera(data, "camera_rear")
                    elif sensor_type == "imu":
                        self._publish_imu(data)
                    elif sensor_type == "gnss":
                        self._publish_gnss(data)

                except queue.Empty:
                    break

        # Also publish vehicle velocity
        if self.vehicle:
            self._publish_velocity()

    def _get_header(self, timestamp: float) -> Header:
        """Create ROS2 header from CARLA timestamp."""
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "base_link"
        return header

    def _publish_lidar(self, data):
        """Convert and publish LiDAR data."""
        # Convert CARLA lidar data to PointCloud2
        points = np.frombuffer(data.raw_data, dtype=np.float32)
        points = np.reshape(points, (int(points.shape[0] / 4), 4))

        msg = PointCloud2()
        msg.header = self._get_header(data.timestamp)
        msg.header.frame_id = "lidar_top_base_link"

        msg.height = 1
        msg.width = len(points)
        msg.is_dense = True
        msg.is_bigendian = False

        # Define fields
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(
                name="intensity", offset=12, datatype=PointField.FLOAT32, count=1
            ),
        ]

        msg.point_step = 16
        msg.row_step = msg.point_step * msg.width
        msg.data = points.tobytes()

        self.publishers["lidar"].publish(msg)

    def _publish_camera(self, data, camera_name: str):
        """Convert and publish camera data."""
        # Convert CARLA image to ROS2 Image
        array = np.frombuffer(data.raw_data, dtype=np.uint8)
        array = np.reshape(array, (data.height, data.width, 4))  # BGRA
        array = array[:, :, :3]  # Remove alpha channel, keep BGR

        msg = Image()
        msg.header = self._get_header(data.timestamp)
        msg.header.frame_id = f"{camera_name}/camera_link"
        msg.height = data.height
        msg.width = data.width
        msg.encoding = "bgr8"
        msg.is_bigendian = False
        msg.step = msg.width * 3
        msg.data = array.tobytes()

        self.publishers[camera_name].publish(msg)

    def _publish_imu(self, data):
        """Convert and publish IMU data."""
        msg = Imu()
        msg.header = self._get_header(data.timestamp)
        msg.header.frame_id = "imu_link"

        # Convert CARLA IMU data
        msg.linear_acceleration.x = data.accelerometer.x
        msg.linear_acceleration.y = data.accelerometer.y
        msg.linear_acceleration.z = data.accelerometer.z

        msg.angular_velocity.x = data.gyroscope.x
        msg.angular_velocity.y = data.gyroscope.y
        msg.angular_velocity.z = data.gyroscope.z

        # Set covariances (using default values)
        msg.orientation_covariance[0] = -1  # Orientation not available
        msg.angular_velocity_covariance[0] = 0.01
        msg.angular_velocity_covariance[4] = 0.01
        msg.angular_velocity_covariance[8] = 0.01
        msg.linear_acceleration_covariance[0] = 0.01
        msg.linear_acceleration_covariance[4] = 0.01
        msg.linear_acceleration_covariance[8] = 0.01

        self.publishers["imu"].publish(msg)

    def _publish_gnss(self, data):
        """Convert and publish GNSS data."""
        msg = NavSatFix()
        msg.header = self._get_header(data.timestamp)
        msg.header.frame_id = "gnss_link"

        msg.latitude = data.latitude
        msg.longitude = data.longitude
        msg.altitude = data.altitude

        # Set covariances (using default values)
        msg.position_covariance[0] = 1.0
        msg.position_covariance[4] = 1.0
        msg.position_covariance[8] = 1.0
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        self.publishers["gnss"].publish(msg)

    def _publish_velocity(self):
        """Publish vehicle velocity."""
        velocity = self.vehicle.get_velocity()

        msg = TwistWithCovarianceStamped()
        msg.header = self._get_header(time.time())
        msg.header.frame_id = "base_link"

        # Convert CARLA velocity (m/s) to ROS
        msg.twist.twist.linear.x = velocity.x
        msg.twist.twist.linear.y = velocity.y
        msg.twist.twist.linear.z = velocity.z

        # Get angular velocity
        angular_velocity = self.vehicle.get_angular_velocity()
        msg.twist.twist.angular.x = np.radians(angular_velocity.x)
        msg.twist.twist.angular.y = np.radians(angular_velocity.y)
        msg.twist.twist.angular.z = np.radians(angular_velocity.z)

        # Set covariances
        msg.twist.covariance[0] = 0.01  # x
        msg.twist.covariance[7] = 0.01  # y
        msg.twist.covariance[14] = 0.01  # z
        msg.twist.covariance[21] = 0.01  # rot_x
        msg.twist.covariance[28] = 0.01  # rot_y
        msg.twist.covariance[35] = 0.01  # rot_z

        self.publishers["velocity"].publish(msg)

    def destroy(self):
        """Clean up resources."""
        for sensor in self.sensors:
            sensor.stop()
            sensor.destroy()
        self.get_logger().info("Bridge shutdown complete")


def main():
    parser = argparse.ArgumentParser(description="CARLA to ROS2 sensor bridge")
    parser.add_argument(
        "--host", default=None, help="CARLA server host (overrides ROS param)"
    )
    parser.add_argument(
        "--port", type=int, default=None, help="CARLA server port (overrides ROS param)"
    )
    parser.add_argument(
        "--role-name",
        default=None,
        help="Vehicle role name to bridge (overrides ROS param)",
    )

    args = parser.parse_args()

    rclpy.init()

    try:
        # Pass None values so ROS params can be used if not overridden
        bridge = CarlaSensorBridge(
            host=args.host,
            port=args.port,
            role_name=args.role_name.replace('-', '_') if args.role_name else None
        )
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        if "bridge" in locals():
            bridge.destroy()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
