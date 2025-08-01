#!/usr/bin/env python3
"""
CARLA Parameter Extraction Module

This module provides classes to extract vehicle and sensor parameters from CARLA
actors for dynamic ROS2 configuration generation.
"""

import math
from typing import Dict, Any, List, Optional, Tuple
import numpy as np

try:
    import carla
except ImportError:
    print("Warning: CARLA Python API not found. Some features may be limited.")
    carla = None


class VehicleParameterExtractor:
    """Extracts vehicle parameters from CARLA actor"""

    def __init__(self, vehicle_actor: "carla.Vehicle"):
        """
        Initialize the vehicle parameter extractor.

        Args:
            vehicle_actor: CARLA vehicle actor instance
        """
        if not vehicle_actor or not hasattr(vehicle_actor, "get_physics_control"):
            raise ValueError("Invalid vehicle actor provided")

        self.vehicle = vehicle_actor
        self._physics_control = None
        self._bounding_box = None

    @property
    def physics_control(self) -> "carla.VehiclePhysicsControl":
        """Lazy load physics control"""
        if self._physics_control is None:
            self._physics_control = self.vehicle.get_physics_control()
        return self._physics_control

    @property
    def bounding_box(self) -> "carla.BoundingBox":
        """Lazy load bounding box"""
        if self._bounding_box is None:
            self._bounding_box = self.vehicle.bounding_box
        return self._bounding_box

    def extract_wheel_parameters(self) -> Dict[str, Any]:
        """
        Extract wheel-related parameters from the vehicle.

        Returns:
            Dictionary containing wheel parameters:
            - wheel_radius: Wheel radius in meters
            - wheel_width: Wheel width in meters (estimated)
            - wheel_base: Distance between front and rear axles in meters
            - wheel_tread: Distance between left and right wheels in meters
            - max_steer_angle: Maximum steering angle in radians
        """
        wheels = self.physics_control.wheels

        if not wheels or len(wheels) < 4:
            raise ValueError("Expected 4 wheels in vehicle physics control")

        # CARLA 0.10.0 WheelPhysicsControl has 'offset' attribute for position
        # Index mapping: 0=FL, 1=FR, 2=BL, 3=BR
        front_left = wheels[0]
        front_right = wheels[1]
        back_left = wheels[2]
        back_right = wheels[3]
        
        # Get wheel positions from offset
        fl_offset = front_left.offset
        fr_offset = front_right.offset
        bl_offset = back_left.offset
        br_offset = back_right.offset
        
        # Calculate wheel base (distance between front and rear axles)
        # Offset.x is the forward/backward position
        front_x = (fl_offset.x + fr_offset.x) / 2.0 / 100.0  # cm to m
        rear_x = (bl_offset.x + br_offset.x) / 2.0 / 100.0  # cm to m
        wheel_base = abs(front_x - rear_x)
        
        # Calculate wheel tread (distance between left and right wheels)
        # Offset.y is the left/right position
        left_y = (fl_offset.y + bl_offset.y) / 2.0 / 100.0  # cm to m
        right_y = (fr_offset.y + br_offset.y) / 2.0 / 100.0  # cm to m
        wheel_tread = abs(left_y - right_y)
        
        # Get wheel radius (all wheels should have same radius)
        wheel_radius = front_left.wheel_radius / 100.0  # cm to m
        
        # Get maximum steer angle (from front wheels)
        max_steer_angle = max(front_left.max_steer_angle, front_right.max_steer_angle)
        max_steer_angle = math.radians(max_steer_angle)  # degrees to radians

        return {
            "wheel_radius": wheel_radius,
            "wheel_width": front_left.wheel_width / 100.0,  # cm to m
            "wheel_base": wheel_base,
            "wheel_tread": wheel_tread,
            "max_steer_angle": max_steer_angle,
        }

    def extract_vehicle_dimensions(self) -> Dict[str, float]:
        """
        Extract vehicle dimension parameters.

        Returns:
            Dictionary containing:
            - vehicle_length: Total vehicle length in meters
            - vehicle_width: Total vehicle width in meters
            - vehicle_height: Total vehicle height in meters
            - front_overhang: Distance from front axle to vehicle front in meters
            - rear_overhang: Distance from rear axle to vehicle rear in meters
            - left_overhang: Distance from left wheels to vehicle left side in meters
            - right_overhang: Distance from right wheels to vehicle right side in meters
        """
        extent = self.bounding_box.extent

        # Convert from cm to m
        length = extent.x * 2.0 / 100.0
        width = extent.y * 2.0 / 100.0
        height = extent.z * 2.0 / 100.0

        # Get wheel parameters for overhang calculations
        wheel_params = self.extract_wheel_parameters()
        wheels = self.physics_control.wheels

        # Find extreme wheel positions from wheel offsets
        # CARLA 0.10.0 uses offset attribute
        front_wheel_x = max(w.offset.x for w in wheels) / 100.0  # cm to m
        rear_wheel_x = min(w.offset.x for w in wheels) / 100.0  # cm to m

        # Calculate overhangs
        # Note: CARLA uses center of bounding box as origin
        front_overhang = (length / 2.0) - front_wheel_x
        rear_overhang = (length / 2.0) + abs(rear_wheel_x)

        # Lateral overhangs (approximate)
        # Assume wheels are centered on track width
        lateral_clearance = (width - wheel_params["wheel_tread"]) / 2.0
        left_overhang = lateral_clearance / 2.0
        right_overhang = lateral_clearance / 2.0

        return {
            "vehicle_length": length,
            "vehicle_width": width,
            "vehicle_height": height,
            "front_overhang": front_overhang,
            "rear_overhang": rear_overhang,
            "left_overhang": left_overhang,
            "right_overhang": right_overhang,
        }

    def extract_physics_parameters(self) -> Dict[str, Any]:
        """
        Extract vehicle physics parameters.

        Returns:
            Dictionary containing:
            - mass: Vehicle mass in kg
            - drag_coefficient: Aerodynamic drag coefficient
            - center_of_mass: Center of mass position relative to vehicle origin
            - max_rpm: Maximum engine RPM
            - damping rates and other physics parameters
        """
        physics = self.physics_control

        # Convert center of mass from cm to m and from vehicle space
        com = physics.center_of_mass

        # CARLA 0.10.0 VehiclePhysicsControl attributes
        return {
            "mass": physics.mass,  # kg
            "drag_coefficient": physics.drag_coefficient,
            "center_of_mass": {
                "x": com.x / 100.0,  # cm to m
                "y": com.y / 100.0,  # cm to m  
                "z": com.z / 100.0,  # cm to m
            },
            "max_rpm": physics.max_rpm,
            "inertia_tensor_scale": {
                "x": physics.inertia_tensor_scale.x,
                "y": physics.inertia_tensor_scale.y,
                "z": physics.inertia_tensor_scale.z,
            },
            "rev_down_rate": physics.rev_down_rate,
            "differential_type": physics.differential_type,
            "front_rear_split": physics.front_rear_split,
            "use_automatic_gears": physics.use_automatic_gears,
            "gear_change_time": physics.gear_change_time,
            "final_ratio": physics.final_ratio,
        }

    def extract_all_parameters(self) -> Dict[str, Any]:
        """
        Extract all vehicle parameters.

        Returns:
            Dictionary containing all vehicle parameters organized by category
        """
        wheel_params = self.extract_wheel_parameters()
        dimension_params = self.extract_vehicle_dimensions()
        physics_params = self.extract_physics_parameters()

        # Combine for Autoware vehicle_info format
        vehicle_info = {
            "wheel_radius": wheel_params["wheel_radius"],
            "wheel_width": wheel_params["wheel_width"],
            "wheel_base": wheel_params["wheel_base"],
            "wheel_tread": wheel_params["wheel_tread"],
            "front_overhang": dimension_params["front_overhang"],
            "rear_overhang": dimension_params["rear_overhang"],
            "left_overhang": dimension_params["left_overhang"],
            "right_overhang": dimension_params["right_overhang"],
            "vehicle_height": dimension_params["vehicle_height"],
            "max_steer_angle": wheel_params["max_steer_angle"],
        }

        # Create simulator model parameters
        simulator_model = self._create_simulator_model_params(
            physics_params, wheel_params
        )

        return {
            "vehicle_info": vehicle_info,
            "simulator_model": simulator_model,
            "physics": physics_params,
            "dimensions": dimension_params,
            "wheels": wheel_params,
        }

    def _create_simulator_model_params(
        self, physics_params: Dict, wheel_params: Dict
    ) -> Dict[str, Any]:
        """
        Create simulator model parameters based on physics data.

        Args:
            physics_params: Physics parameters dictionary
            wheel_params: Wheel parameters dictionary

        Returns:
            Dictionary with simulator model parameters
        """
        # Estimate acceleration limits based on mass and engine
        mass = physics_params.get("mass", 1500.0)
        max_rpm = physics_params.get("max_rpm", 6000.0)

        # Simple estimation of max acceleration
        max_acceleration = min(7.0, 5000.0 / mass)

        return {
            "simulated_frame_id": "base_link",
            "origin_frame_id": "map",
            "vehicle_model_type": "DELAY_STEER_ACC_GEARED",
            "initialize_source": "INITIAL_POSE_TOPIC",
            "timer_sampling_time_ms": 25,
            "add_measurement_noise": False,
            "vel_lim": 50.0,  # m/s
            "vel_rate_lim": max_acceleration,
            "steer_lim": wheel_params["max_steer_angle"],
            "steer_rate_lim": 5.0,  # rad/s (estimated)
            "acc_time_delay": 0.1,
            "acc_time_constant": 0.1,
            "steer_time_delay": 0.24,
            "steer_time_constant": 0.27,
            "steer_dead_band": 0.0,
            "x_stddev": 0.0001,
            "y_stddev": 0.0001,
            "enable_road_slope_simulation": True,
        }


class SensorParameterExtractor:
    """Extracts sensor parameters from CARLA sensor actors"""

    def __init__(self, world: "carla.World", vehicle_actor: "carla.Vehicle"):
        """
        Initialize the sensor parameter extractor.

        Args:
            world: CARLA world instance
            vehicle_actor: CARLA vehicle actor that sensors are attached to
        """
        self.world = world
        self.vehicle = vehicle_actor
        self._sensors = None

    @property
    def sensors(self) -> List["carla.Actor"]:
        """Get all sensors attached to vehicle"""
        if self._sensors is None:
            all_actors = self.world.get_actors()
            self._sensors = []

            for actor in all_actors:
                # Check if actor is a sensor attached to our vehicle
                if (
                    hasattr(actor, "parent")
                    and actor.parent
                    and actor.parent.id == self.vehicle.id
                    and "sensor" in actor.type_id
                ):
                    self._sensors.append(actor)

        return self._sensors

    def extract_camera_parameters(self, camera: "carla.Actor") -> Dict[str, Any]:
        """
        Extract camera-specific parameters.

        Args:
            camera: CARLA camera sensor actor

        Returns:
            Dictionary with camera parameters
        """
        attributes = camera.attributes

        return {
            "sensor_type": "camera",
            "type_id": camera.type_id,
            "image_width": int(attributes.get("image_size_x", 800)),
            "image_height": int(attributes.get("image_size_y", 600)),
            "fov": float(attributes.get("fov", 90.0)),
            "sensor_tick": float(attributes.get("sensor_tick", 0.0)),
            "gamma": float(attributes.get("gamma", 2.2)),
            "shutter_speed": float(attributes.get("shutter_speed", 200.0)),
            "iso": float(attributes.get("iso", 100.0)),
            "fstop": float(attributes.get("fstop", 1.4)),
            "min_fstop": float(attributes.get("min_fstop", 1.2)),
            "blade_count": int(attributes.get("blade_count", 5)),
            "exposure_mode": attributes.get("exposure_mode", "manual"),
            "exposure_compensation": float(
                attributes.get("exposure_compensation", 0.0)
            ),
            "exposure_min_bright": float(attributes.get("exposure_min_bright", 10.0)),
            "exposure_max_bright": float(attributes.get("exposure_max_bright", 12.0)),
            "exposure_speed_up": float(attributes.get("exposure_speed_up", 3.0)),
            "exposure_speed_down": float(attributes.get("exposure_speed_down", 1.0)),
            "calibration_constant": float(attributes.get("calibration_constant", 16.0)),
            "focal_distance": float(attributes.get("focal_distance", 1000.0)),
            "blur_amount": float(attributes.get("blur_amount", 1.0)),
            "blur_radius": float(attributes.get("blur_radius", 0.0)),
            "motion_blur_intensity": float(
                attributes.get("motion_blur_intensity", 0.45)
            ),
            "motion_blur_max_distortion": float(
                attributes.get("motion_blur_max_distortion", 0.35)
            ),
            "motion_blur_min_object_screen_size": float(
                attributes.get("motion_blur_min_object_screen_size", 0.1)
            ),
            "lens_flare_intensity": float(attributes.get("lens_flare_intensity", 0.1)),
            "bloom_intensity": float(attributes.get("bloom_intensity", 0.675)),
            "lens_x_size": float(attributes.get("lens_x_size", 0.08)),
            "lens_y_size": float(attributes.get("lens_y_size", 0.08)),
            "lens_k": float(attributes.get("lens_k", -1.0)),
            "lens_kcube": float(attributes.get("lens_kcube", 0.0)),
            "chromatic_aberration_intensity": float(
                attributes.get("chromatic_aberration_intensity", 0.0)
            ),
            "chromatic_aberration_offset": float(
                attributes.get("chromatic_aberration_offset", 0.0)
            ),
            "enable_postprocess_effects": attributes.get(
                "enable_postprocess_effects", "true"
            ).lower()
            == "true",
        }

    def extract_lidar_parameters(self, lidar: "carla.Actor") -> Dict[str, Any]:
        """
        Extract LiDAR-specific parameters.

        Args:
            lidar: CARLA LiDAR sensor actor

        Returns:
            Dictionary with LiDAR parameters
        """
        attributes = lidar.attributes

        return {
            "sensor_type": "lidar",
            "type_id": lidar.type_id,
            "channels": int(attributes.get("channels", 32)),
            "range": float(attributes.get("range", 100.0)),
            "points_per_second": int(attributes.get("points_per_second", 500000)),
            "rotation_frequency": float(attributes.get("rotation_frequency", 10.0)),
            "upper_fov": float(attributes.get("upper_fov", 10.0)),
            "lower_fov": float(attributes.get("lower_fov", -30.0)),
            "horizontal_fov": float(attributes.get("horizontal_fov", 360.0)),
            "atmosphere_attenuation_rate": float(
                attributes.get("atmosphere_attenuation_rate", 0.004)
            ),
            "dropoff_general_rate": float(attributes.get("dropoff_general_rate", 0.45)),
            "dropoff_intensity_limit": float(
                attributes.get("dropoff_intensity_limit", 0.8)
            ),
            "dropoff_zero_intensity": float(
                attributes.get("dropoff_zero_intensity", 0.4)
            ),
            "noise_stddev": float(attributes.get("noise_stddev", 0.0)),
        }

    def extract_imu_parameters(self, imu: "carla.Actor") -> Dict[str, Any]:
        """
        Extract IMU-specific parameters.

        Args:
            imu: CARLA IMU sensor actor

        Returns:
            Dictionary with IMU parameters
        """
        attributes = imu.attributes

        return {
            "sensor_type": "imu",
            "type_id": imu.type_id,
            "noise_accel_stddev_x": float(attributes.get("noise_accel_stddev_x", 0.0)),
            "noise_accel_stddev_y": float(attributes.get("noise_accel_stddev_y", 0.0)),
            "noise_accel_stddev_z": float(attributes.get("noise_accel_stddev_z", 0.0)),
            "noise_gyro_stddev_x": float(attributes.get("noise_gyro_stddev_x", 0.0)),
            "noise_gyro_stddev_y": float(attributes.get("noise_gyro_stddev_y", 0.0)),
            "noise_gyro_stddev_z": float(attributes.get("noise_gyro_stddev_z", 0.0)),
            "noise_gyro_bias_x": float(attributes.get("noise_gyro_bias_x", 0.0)),
            "noise_gyro_bias_y": float(attributes.get("noise_gyro_bias_y", 0.0)),
            "noise_gyro_bias_z": float(attributes.get("noise_gyro_bias_z", 0.0)),
        }

    def extract_gnss_parameters(self, gnss: "carla.Actor") -> Dict[str, Any]:
        """
        Extract GNSS-specific parameters.

        Args:
            gnss: CARLA GNSS sensor actor

        Returns:
            Dictionary with GNSS parameters
        """
        attributes = gnss.attributes

        return {
            "sensor_type": "gnss",
            "type_id": gnss.type_id,
            "noise_alt_stddev": float(attributes.get("noise_alt_stddev", 0.0)),
            "noise_lat_stddev": float(attributes.get("noise_lat_stddev", 0.0)),
            "noise_lon_stddev": float(attributes.get("noise_lon_stddev", 0.0)),
            "noise_alt_bias": float(attributes.get("noise_alt_bias", 0.0)),
            "noise_lat_bias": float(attributes.get("noise_lat_bias", 0.0)),
            "noise_lon_bias": float(attributes.get("noise_lon_bias", 0.0)),
        }

    def extract_sensor_transform(self, sensor: "carla.Actor") -> Dict[str, float]:
        """
        Extract sensor position and orientation relative to vehicle.

        Args:
            sensor: CARLA sensor actor

        Returns:
            Dictionary with transform parameters in ROS coordinate system
        """
        transform = sensor.get_transform()
        vehicle_transform = self.vehicle.get_transform()

        # Calculate relative transform
        # Get sensor location in world coordinates
        sensor_loc = transform.location
        vehicle_loc = vehicle_transform.location

        # Calculate relative position
        relative_x = sensor_loc.x - vehicle_loc.x
        relative_y = sensor_loc.y - vehicle_loc.y
        relative_z = sensor_loc.z - vehicle_loc.z

        # Rotate to vehicle frame
        yaw = math.radians(vehicle_transform.rotation.yaw)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        rel_x_vehicle = relative_x * cos_yaw + relative_y * sin_yaw
        rel_y_vehicle = -relative_x * sin_yaw + relative_y * cos_yaw
        rel_z_vehicle = relative_z

        # Convert from CARLA (UE4) to ROS coordinate system
        # CARLA: X-forward, Y-right, Z-up (left-handed)
        # ROS: X-forward, Y-left, Z-up (right-handed)
        return {
            "x": rel_x_vehicle / 100.0,  # cm to m
            "y": -rel_y_vehicle / 100.0,  # flip Y axis and convert to m
            "z": rel_z_vehicle / 100.0,  # cm to m
            "roll": math.radians(transform.rotation.roll),
            "pitch": math.radians(-transform.rotation.pitch),  # flip pitch
            "yaw": math.radians(-transform.rotation.yaw),  # flip yaw
        }

    def extract_all_sensor_parameters(self) -> Dict[str, Any]:
        """
        Extract parameters for all sensors attached to the vehicle.

        Returns:
            Dictionary mapping sensor names to their parameters
        """
        sensor_configs = {}

        for sensor in self.sensors:
            # Get sensor role name or create one from ID
            sensor_name = sensor.attributes.get("role_name", f"sensor_{sensor.id}")

            # Extract base parameters
            config = {
                "sensor_id": sensor.id,
                "type_id": sensor.type_id,
                "role_name": sensor_name,
                "transform": self.extract_sensor_transform(sensor),
            }

            # Add sensor-specific parameters
            if "camera" in sensor.type_id:
                config.update(self.extract_camera_parameters(sensor))
                # Generate frame_id for camera
                config["frame_id"] = f"{sensor_name}/camera_link"
            elif "lidar" in sensor.type_id:
                config.update(self.extract_lidar_parameters(sensor))
                # Generate frame_id for lidar
                config["frame_id"] = f"{sensor_name}_base_link"
            elif "imu" in sensor.type_id:
                config.update(self.extract_imu_parameters(sensor))
                # Generate frame_id for IMU
                config["frame_id"] = f"{sensor_name}_link"
            elif "gnss" in sensor.type_id:
                config.update(self.extract_gnss_parameters(sensor))
                # Generate frame_id for GNSS
                config["frame_id"] = f"{sensor_name}_link"
            else:
                config["sensor_type"] = "unknown"
                config["frame_id"] = f"{sensor_name}_link"

            sensor_configs[sensor_name] = config

        return sensor_configs


class CoordinateTransformer:
    """Handles coordinate system transformations between CARLA and ROS"""

    @staticmethod
    def carla_to_ros_position(carla_vector: "carla.Vector3D") -> Dict[str, float]:
        """
        Convert CARLA position to ROS position.

        Args:
            carla_vector: CARLA Vector3D position

        Returns:
            Dictionary with x, y, z in ROS coordinate system (meters)
        """
        return {
            "x": carla_vector.x / 100.0,  # cm to m
            "y": -carla_vector.y / 100.0,  # flip Y axis, cm to m
            "z": carla_vector.z / 100.0,  # cm to m
        }

    @staticmethod
    def carla_to_ros_rotation(carla_rotation: "carla.Rotation") -> Dict[str, float]:
        """
        Convert CARLA rotation to ROS rotation.

        Args:
            carla_rotation: CARLA Rotation (in degrees)

        Returns:
            Dictionary with roll, pitch, yaw in ROS coordinate system (radians)
        """
        return {
            "roll": math.radians(carla_rotation.roll),
            "pitch": math.radians(-carla_rotation.pitch),  # flip pitch
            "yaw": math.radians(-carla_rotation.yaw),  # flip yaw
        }

    @staticmethod
    def calculate_quaternion(roll: float, pitch: float, yaw: float) -> Dict[str, float]:
        """
        Calculate quaternion from Euler angles.

        Args:
            roll: Roll angle in radians
            pitch: Pitch angle in radians
            yaw: Yaw angle in radians

        Returns:
            Dictionary with x, y, z, w quaternion components
        """
        # Calculate quaternion components
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        return {
            "x": sr * cp * cy - cr * sp * sy,
            "y": cr * sp * cy + sr * cp * sy,
            "z": cr * cp * sy - sr * sp * cy,
            "w": cr * cp * cy + sr * sp * sy,
        }

    @staticmethod
    def transform_velocity(
        velocity: "carla.Vector3D", yaw: float
    ) -> Tuple[float, float, float]:
        """
        Transform velocity from world to vehicle frame.

        Args:
            velocity: CARLA velocity vector in world frame
            yaw: Vehicle yaw angle in radians

        Returns:
            Tuple of (vx, vy, vz) in vehicle frame (m/s)
        """
        # Convert to m/s
        vx_world = velocity.x / 100.0
        vy_world = velocity.y / 100.0
        vz_world = velocity.z / 100.0

        # Rotate to vehicle frame
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        vx_vehicle = vx_world * cos_yaw + vy_world * sin_yaw
        vy_vehicle = -vx_world * sin_yaw + vy_world * cos_yaw

        # Apply ROS coordinate system (flip Y)
        return vx_vehicle, -vy_vehicle, vz_world
