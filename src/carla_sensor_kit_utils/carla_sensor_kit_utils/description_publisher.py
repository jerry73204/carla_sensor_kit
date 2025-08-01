#!/usr/bin/env python3
"""
Dynamic Description Publisher Node

This node publishes robot and sensor descriptions directly to ROS topics,
bypassing the need for description files. It integrates with CARLA to
generate descriptions at runtime.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from std_msgs.msg import String
from rcl_interfaces.msg import ParameterDescriptor
import yaml
import json

try:
    import carla
except ImportError:
    carla = None

from carla_sensor_kit_utils.parameter_extractor import (
    VehicleParameterExtractor,
    SensorParameterExtractor,
)
from carla_sensor_kit_utils.urdf_generator import DynamicURDFGenerator


class DescriptionPublisher(Node):
    """Publishes robot and sensor descriptions as ROS topics"""

    def __init__(self):
        super().__init__("description_publisher")

        # Declare parameters
        self.declare_parameter("carla_host", "localhost")
        self.declare_parameter("carla_port", 2000)
        self.declare_parameter("vehicle_role_name", "ego_vehicle")
        self.declare_parameter("publish_rate", 1.0)  # Hz

        # QoS profile for latching behavior
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )

        # Publishers
        self.robot_desc_pub = self.create_publisher(
            String, "/robot_description", qos_profile
        )

        self.sensor_calibration_pub = self.create_publisher(
            String, "/sensor_kit_calibration", qos_profile
        )

        self.vehicle_info_pub = self.create_publisher(
            String, "/vehicle_info", qos_profile
        )

        # Initialize CARLA connection
        self.carla_client = None
        self.carla_world = None
        self.vehicle = None

        # Try to connect to CARLA
        if self._connect_to_carla():
            self._find_vehicle()

            if self.vehicle:
                # Generate and publish descriptions
                self._generate_and_publish_descriptions()

                # Create timer for periodic updates (if needed)
                timer_period = 1.0 / self.get_parameter("publish_rate").value
                self.timer = self.create_timer(timer_period, self._timer_callback)
            else:
                self.get_logger().error("Vehicle not found in CARLA")
        else:
            self.get_logger().error("Failed to connect to CARLA")

    def _connect_to_carla(self):
        """Connect to CARLA simulator"""
        if not carla:
            self.get_logger().error("CARLA Python API not available")
            return False

        try:
            host = self.get_parameter("carla_host").value
            port = self.get_parameter("carla_port").value

            self.carla_client = carla.Client(host, port)
            self.carla_client.set_timeout(10.0)
            self.carla_world = self.carla_client.get_world()

            self.get_logger().info(f"Connected to CARLA at {host}:{port}")
            return True

        except Exception as e:
            self.get_logger().error(f"Failed to connect to CARLA: {e}")
            return False

    def _find_vehicle(self):
        """Find vehicle in CARLA by role name"""
        if not self.carla_world:
            return

        role_name = self.get_parameter("vehicle_role_name").value
        actors = self.carla_world.get_actors().filter("vehicle.*")

        for actor in actors:
            if actor.attributes.get("role_name") == role_name:
                self.vehicle = actor
                self.get_logger().info(f"Found vehicle: {role_name}")
                return

        self.get_logger().warning(f'Vehicle with role_name "{role_name}" not found')

    def _generate_and_publish_descriptions(self):
        """Generate descriptions from CARLA and publish to topics"""
        if not self.vehicle:
            return

        try:
            # Extract parameters
            vehicle_extractor = VehicleParameterExtractor(self.vehicle)
            sensor_extractor = SensorParameterExtractor(self.carla_world, self.vehicle)

            vehicle_params = vehicle_extractor.extract_all_parameters()
            sensor_params = sensor_extractor.extract_all_sensor_parameters()

            # Generate URDF
            urdf_generator = DynamicURDFGenerator()
            robot_description = urdf_generator.generate_urdf_string(
                vehicle_params, sensor_params
            )

            # Publish robot description
            robot_desc_msg = String()
            robot_desc_msg.data = robot_description
            self.robot_desc_pub.publish(robot_desc_msg)
            self.get_logger().info("Published robot description")

            # Generate and publish sensor calibration
            sensor_calibration = self._generate_sensor_calibration(sensor_params)
            calib_msg = String()
            calib_msg.data = yaml.dump(sensor_calibration)
            self.sensor_calibration_pub.publish(calib_msg)
            self.get_logger().info("Published sensor calibration")

            # Publish vehicle info
            vehicle_info_msg = String()
            vehicle_info_msg.data = yaml.dump(
                {"/**": {"ros__parameters": vehicle_params["vehicle_info"]}}
            )
            self.vehicle_info_pub.publish(vehicle_info_msg)
            self.get_logger().info("Published vehicle info")

            # Also set as parameters for compatibility
            self.set_parameters(
                [
                    rclpy.parameter.Parameter(
                        "robot_description",
                        rclpy.parameter.Parameter.Type.STRING,
                        robot_description,
                    )
                ]
            )

        except Exception as e:
            self.get_logger().error(f"Failed to generate descriptions: {e}")

    def _generate_sensor_calibration(self, sensor_params):
        """Generate sensor calibration in Autoware format"""
        calibration = {}

        for sensor_name, sensor_config in sensor_params.items():
            transform = sensor_config.get("transform", {})

            # Convert to Autoware calibration format
            calibration[sensor_name] = {
                "x": transform.get("x", 0.0),
                "y": transform.get("y", 0.0),
                "z": transform.get("z", 0.0),
                "roll": transform.get("roll", 0.0),
                "pitch": transform.get("pitch", 0.0),
                "yaw": transform.get("yaw", 0.0),
            }

        return calibration

    def _timer_callback(self):
        """Periodic callback to republish descriptions"""
        # Descriptions are latched, so we don't need to republish frequently
        # This is here in case we want to update descriptions dynamically
        pass


def main(args=None):
    rclpy.init(args=args)

    node = DescriptionPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
