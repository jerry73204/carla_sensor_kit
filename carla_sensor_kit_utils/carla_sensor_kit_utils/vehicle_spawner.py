#!/usr/bin/env python3
"""
ROS 2 node for spawning vehicles with sensors in CARLA simulator.
Reads sensor configurations from carla_sensor_kit_description package.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory

import carla
import yaml
import numpy as np
from pathlib import Path
from typing import Optional

# Import sensor config loader from same package
from .sensor_config_loader import SensorConfigLoader


class VehicleSpawner(Node):
    """ROS 2 node that spawns vehicles with sensors in CARLA."""

    def __init__(self):
        super().__init__("carla_vehicle_spawner")

        # Declare parameters
        self.declare_parameter("host", "localhost")
        self.declare_parameter("port", 2000)
        self.declare_parameter("timeout", 10.0)
        self.declare_parameter("vehicle_type", "vehicle.tesla.model3")
        self.declare_parameter("vehicle_role_name", "hero")
        self.declare_parameter("spawn_point_index", -1)  # -1 for random
        self.declare_parameter("autopilot", True)
        self.declare_parameter("enable_sensors", True)
        self.declare_parameter(
            "auto_unique_name", False
        )  # Auto-append number to role_name

        # Get parameters
        self.host = self.get_parameter("host").value
        self.port = self.get_parameter("port").value
        self.timeout = self.get_parameter("timeout").value
        self.vehicle_type = self.get_parameter("vehicle_type").value
        self.vehicle_role_name = self.get_parameter("vehicle_role_name").value
        self.spawn_point_index = self.get_parameter("spawn_point_index").value
        self.autopilot = self.get_parameter("autopilot").value
        self.enable_sensors = self.get_parameter("enable_sensors").value
        self.auto_unique_name = self.get_parameter("auto_unique_name").value

        # Initialize CARLA client
        self.client = None
        self.world = None
        self.spawned_actors = []

        # Load sensor configurations
        self.sensor_configs = self._load_sensor_configs()

        # Publisher for status
        self.status_publisher = self.create_publisher(String, "~/status", 10)

        # Initialize spawning status
        self.spawning_complete = False

        # Connect to CARLA and spawn vehicle
        self._connect_and_spawn()

        # Timer for status updates - commented out since we're exiting immediately
        # self.create_timer(1.0, self._publish_status)

    def _load_sensor_configs(self) -> dict:
        """Load sensor configurations from YAML files."""
        try:
            # Get package path
            sensor_kit_path = get_package_share_directory(
                "carla_sensor_kit_description"
            )
            config_path = Path(sensor_kit_path) / "config"

            # Load calibration files
            with open(config_path / "sensors_calibration.yaml", "r") as f:
                sensors_calib = yaml.safe_load(f)

            with open(config_path / "sensor_kit_calibration.yaml", "r") as f:
                kit_calib = yaml.safe_load(f)

            self.get_logger().info(f"Loaded sensor configurations from {config_path}")

            return {
                "sensors_calibration": sensors_calib,
                "sensor_kit_calibration": kit_calib,
            }

        except Exception as e:
            self.get_logger().error(f"Failed to load sensor configurations: {e}")
            return {}

    def _to_carla_transform(
        self, x: float, y: float, z: float, roll: float, pitch: float, yaw: float
    ) -> carla.Transform:
        """Convert ROS coordinate system to CARLA Transform."""
        # ROS uses right-hand rule (x-forward, y-left, z-up)
        # CARLA uses left-hand rule (x-forward, y-right, z-up)
        carla_y = -y

        # Convert radians to degrees and adjust for coordinate system
        carla_roll = np.degrees(roll)
        carla_pitch = np.degrees(pitch)
        carla_yaw = np.degrees(-yaw)

        return carla.Transform(
            carla.Location(x=x, y=carla_y, z=z),
            carla.Rotation(pitch=carla_pitch, yaw=carla_yaw, roll=carla_roll),
        )

    def _get_sensor_transform(self, sensor_name: str) -> Optional[carla.Transform]:
        """Get sensor transform from configuration."""
        if not self.sensor_configs:
            return None

        # Get sensor kit calibration
        kit_calib = self.sensor_configs["sensor_kit_calibration"][
            "sensor_kit_base_link"
        ]
        if sensor_name not in kit_calib:
            self.get_logger().warn(f"Sensor '{sensor_name}' not found in calibration")
            return None

        # Get base_link to sensor_kit transform
        base_to_kit = self.sensor_configs["sensors_calibration"]["base_link"][
            "sensor_kit_base_link"
        ]

        # Get sensor transform relative to sensor kit
        kit_to_sensor = kit_calib[sensor_name]

        # Combine transforms (simplified - assumes small angles)
        combined = {
            "x": base_to_kit["x"] + kit_to_sensor["x"],
            "y": base_to_kit["y"] + kit_to_sensor["y"],
            "z": base_to_kit["z"] + kit_to_sensor["z"],
            "roll": base_to_kit["roll"] + kit_to_sensor["roll"],
            "pitch": base_to_kit["pitch"] + kit_to_sensor["pitch"],
            "yaw": base_to_kit["yaw"] + kit_to_sensor["yaw"],
        }

        return self._to_carla_transform(**combined)

    def _connect_and_spawn(self):
        """Connect to CARLA and spawn vehicle with sensors."""
        try:
            # Connect to CARLA
            self.client = carla.Client(self.host, self.port)
            self.client.set_timeout(self.timeout)
            self.world = self.client.get_world()
            self.get_logger().info(
                f"Connected to CARLA server at {self.host}:{self.port}"
            )

            # Check if a vehicle with this role_name already exists
            existing_vehicles = self.world.get_actors().filter("vehicle.*")
            existing_role_names = set()
            for vehicle in existing_vehicles:
                role_name = vehicle.attributes.get("role_name", "")
                if role_name:
                    existing_role_names.add(role_name)

            # Handle duplicate role_name
            original_role_name = self.vehicle_role_name
            if self.vehicle_role_name in existing_role_names:
                if self.auto_unique_name:
                    # Find a unique name by appending a number
                    counter = 1
                    while f"{original_role_name}_{counter}" in existing_role_names:
                        counter += 1
                    self.vehicle_role_name = f"{original_role_name}_{counter}"
                    self.get_logger().info(
                        f"Role name '{original_role_name}' already exists, using '{self.vehicle_role_name}' instead"
                    )
                else:
                    # Find the existing vehicle with this role_name
                    for vehicle in existing_vehicles:
                        if (
                            vehicle.attributes.get("role_name", "")
                            == self.vehicle_role_name
                        ):
                            self.get_logger().error(
                                f"Vehicle with role_name '{self.vehicle_role_name}' already exists (ID: {vehicle.id})"
                            )
                            break
                    self.get_logger().error(
                        "Please use 'make reset' to clear existing vehicles or use auto_unique_name:=True"
                    )
                    self.spawning_complete = False
                    return

            # Get blueprint library
            blueprint_library = self.world.get_blueprint_library()

            # Spawn vehicle
            vehicle_blueprints = blueprint_library.filter(self.vehicle_type)
            if not vehicle_blueprints:
                # Fallback to any vehicle if specific type not found
                self.get_logger().warn(
                    f"Vehicle type '{self.vehicle_type}' not found, using default"
                )
                vehicle_blueprints = blueprint_library.filter("vehicle.*")

            vehicle_bp = vehicle_blueprints[0]
            vehicle_bp.set_attribute("role_name", self.vehicle_role_name)

            # Get spawn point
            spawn_points = self.world.get_map().get_spawn_points()
            if self.spawn_point_index < 0 or self.spawn_point_index >= len(
                spawn_points
            ):
                spawn_point = np.random.choice(spawn_points)
            else:
                spawn_point = spawn_points[self.spawn_point_index]

            # Spawn vehicle
            vehicle = self.world.spawn_actor(vehicle_bp, spawn_point)
            self.spawned_actors.append(vehicle)
            self.get_logger().info(
                f"Spawned vehicle '{self.vehicle_role_name}' (ID: {vehicle.id}) at {spawn_point.location}"
            )
            self.get_logger().info(
                f"Note: CARLA ROS Bridge will use /carla/vehicle{vehicle.id}/* for topic names"
            )

            # Set autopilot if requested
            if self.autopilot:
                vehicle.set_autopilot(True)
                self.get_logger().info("Autopilot enabled")

            # Spawn sensors if enabled
            if self.enable_sensors:
                self._spawn_sensors(vehicle, blueprint_library)

            # Give CARLA time to initialize ROS2 publishers
            import time

            time.sleep(0.5)

            # Mark spawning as complete
            self.spawning_complete = True

        except Exception as e:
            import traceback

            self.get_logger().error(f"Failed to connect/spawn: {e}")
            self.get_logger().error(f"Traceback: {traceback.format_exc()}")
            self.spawning_complete = False

    def _spawn_sensors(self, vehicle, blueprint_library):
        """Spawn sensors on the vehicle."""
        sensor_configs = [
            # Single LiDAR on top
            (
                "sensor.lidar.ray_cast",
                "lidar_top",
                "velodyne_top_base_link",
                {
                    "channels": "128",
                    "range": "100",
                    "points_per_second": "2200000",
                    "rotation_frequency": "10",
                    "upper_fov": "10",
                    "lower_fov": "-30",
                },
            ),
            # Four cameras: front, back, left, right
            (
                "sensor.camera.rgb",
                "camera_front",
                "camera_front/camera_link",
                {
                    "image_size_x": "800",
                    "image_size_y": "400",
                    "fov": "90",
                    "sensor_tick": "0.033",  # ~30 FPS
                },
            ),
            (
                "sensor.camera.rgb",
                "camera_back",
                "camera_back/camera_link",
                {
                    "image_size_x": "800",
                    "image_size_y": "400",
                    "fov": "90",
                    "sensor_tick": "0.033",  # ~30 FPS
                },
            ),
            (
                "sensor.camera.rgb",
                "camera_left",
                "camera_left/camera_link",
                {
                    "image_size_x": "800",
                    "image_size_y": "400",
                    "fov": "90",
                    "sensor_tick": "0.033",  # ~30 FPS
                },
            ),
            (
                "sensor.camera.rgb",
                "camera_right",
                "camera_right/camera_link",
                {
                    "image_size_x": "800",
                    "image_size_y": "400",
                    "fov": "90",
                    "sensor_tick": "0.033",  # ~30 FPS
                },
            ),
            # IMU with renamed link
            ("sensor.other.imu", "imu", "imu_link", {}),
            # GNSS
            ("sensor.other.gnss", "gnss", "gnss_link", {}),
        ]

        for sensor_type, sensor_name, config_name, attributes in sensor_configs:
            try:
                # Get sensor blueprint
                sensor_bp = blueprint_library.find(sensor_type)
                sensor_bp.set_attribute("role_name", sensor_name)

                # Set ROS name for CARLA ROS bridge
                sensor_bp.set_attribute("ros_name", sensor_name)

                # Enable sensor for ROS2 publishing
                if sensor_bp.has_attribute("sensor_tick"):
                    # Set sensor_tick to 0 for event-based (as fast as possible)
                    sensor_bp.set_attribute("sensor_tick", "0.0")

                # Set additional attributes
                for key, value in attributes.items():
                    if sensor_bp.has_attribute(key):
                        sensor_bp.set_attribute(key, value)

                # Get transform from configuration
                transform = self._get_sensor_transform(config_name)
                if transform is None:
                    transform = carla.Transform()  # Use default if not found

                # Spawn sensor
                sensor = self.world.spawn_actor(sensor_bp, transform, attach_to=vehicle)
                self.spawned_actors.append(sensor)

                self.get_logger().info(
                    f"Spawned {sensor_name} at Location({transform.location.x:.2f}, "
                    f"{transform.location.y:.2f}, {transform.location.z:.2f})"
                )

                # Enable sensor for ROS2 native publishing
                # This is required when using CARLA with --ros2 flag
                try:
                    sensor.enable_for_ros()
                    self.get_logger().info(
                        f"Enabled {sensor_name} for ROS2 native publishing"
                    )
                except AttributeError:
                    # Fallback if enable_for_ros() is not available in this CARLA version
                    self.get_logger().warn(
                        f"sensor.enable_for_ros() not available for {sensor_name}. "
                        "Make sure CARLA is started with --ros2 flag."
                    )

            except Exception as e:
                self.get_logger().error(f"Failed to spawn sensor {sensor_name}: {e}")
                import traceback

                self.get_logger().debug(f"Traceback: {traceback.format_exc()}")

    def _publish_status(self):
        """Publish spawner status."""
        msg = String()
        msg.data = (
            f"Vehicle: {self.vehicle_role_name}, Actors: {len(self.spawned_actors)}"
        )
        self.status_publisher.publish(msg)

    def destroy_actors(self):
        """Destroy all spawned actors."""
        self.get_logger().info("Destroying spawned actors...")
        for actor in self.spawned_actors:
            try:
                actor.destroy()
            except Exception as e:
                self.get_logger().warn(f"Failed to destroy actor: {e}")
        self.spawned_actors.clear()

    def __del__(self):
        """Cleanup on node destruction."""
        # Don't automatically destroy actors when the node is deleted
        # This allows the spawned vehicle to remain in CARLA after the script exits
        pass


def main(args=None):
    rclpy.init(args=args)

    try:
        spawner = VehicleSpawner()

        # Check if spawning was successful
        if hasattr(spawner, "spawning_complete") and spawner.spawning_complete:
            spawner.get_logger().info(
                "Vehicle and sensors spawned successfully. Exiting..."
            )
            # Publish one final status before exiting
            spawner._publish_status()
            # Exit immediately - don't spin
        else:
            # If spawning failed, keep the node running for debugging
            spawner.get_logger().error(
                "Spawning failed or incomplete. Keeping node running for debugging..."
            )
            rclpy.spin(spawner)
    except KeyboardInterrupt:
        pass
    finally:
        if "spawner" in locals():
            # Don't destroy actors on exit - leave them in CARLA
            spawner.get_logger().info("Exiting spawner (actors remain in CARLA)")
            spawner.spawned_actors.clear()  # Clear list without destroying actors
            spawner.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
