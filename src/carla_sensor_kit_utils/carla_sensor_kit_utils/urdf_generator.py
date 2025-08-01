#!/usr/bin/env python3
"""
Dynamic URDF Generator Module

This module provides functionality to generate URDF robot descriptions
dynamically from vehicle and sensor parameters extracted from CARLA.
"""

import math
from typing import Dict, Any, List, Optional
from xml.etree import ElementTree as ET
from xml.dom import minidom

from .parameter_extractor import CoordinateTransformer


class DynamicURDFGenerator:
    """Generates URDF robot descriptions from extracted parameters"""

    def __init__(self):
        """Initialize the URDF generator"""
        self.transformer = CoordinateTransformer()

    def generate_urdf_string(
        self,
        vehicle_params: Dict[str, Any],
        sensor_params: Dict[str, Any],
        pretty_print: bool = True,
    ) -> str:
        """
        Generate complete URDF as XML string.

        Args:
            vehicle_params: Vehicle parameters from VehicleParameterExtractor
            sensor_params: Sensor parameters from SensorParameterExtractor
            pretty_print: Whether to format the XML with indentation

        Returns:
            URDF XML as string
        """
        # Create root robot element
        robot = ET.Element("robot", name="carla_vehicle")

        # Add base_link (vehicle body)
        self._add_base_link(robot, vehicle_params)

        # Add sensor_kit_base_link
        self._add_sensor_kit_link(robot)

        # Add individual sensors
        for sensor_name, sensor_config in sensor_params.items():
            self._add_sensor_link(robot, sensor_name, sensor_config)

        # Convert to string
        if pretty_print:
            return self._prettify_xml(robot)
        else:
            return ET.tostring(robot, encoding="unicode")

    def _add_base_link(self, robot: ET.Element, vehicle_params: Dict[str, Any]):
        """
        Add base_link (vehicle body) to URDF.

        Args:
            robot: Root robot element
            vehicle_params: Vehicle parameters dictionary
        """
        # Extract vehicle info parameters
        vehicle_info = vehicle_params.get("vehicle_info", {})
        dimensions = vehicle_params.get("dimensions", {})

        # Create base_link
        base_link = ET.SubElement(robot, "link", name="base_link")

        # Add visual element
        visual = ET.SubElement(base_link, "visual")

        # Calculate visual origin (center of vehicle bounding box)
        # Base link is at center of rear axle, so offset forward by half wheelbase
        origin_x = vehicle_info.get("wheel_base", 2.5) / 2.0
        origin_z = dimensions.get("vehicle_height", 1.5) / 2.0

        visual_origin = ET.SubElement(visual, "origin")
        visual_origin.set("xyz", f"{origin_x:.3f} 0 {origin_z:.3f}")
        visual_origin.set("rpy", "0 0 0")

        # Add geometry
        geometry = ET.SubElement(visual, "geometry")
        box = ET.SubElement(geometry, "box")

        # Calculate box size
        length = dimensions.get("vehicle_length", 4.5)
        width = dimensions.get("vehicle_width", 1.8)
        height = dimensions.get("vehicle_height", 1.5)

        box.set("size", f"{length:.3f} {width:.3f} {height:.3f}")

        # Add collision (same as visual)
        collision = ET.SubElement(base_link, "collision")
        collision_origin = ET.SubElement(collision, "origin")
        collision_origin.set("xyz", f"{origin_x:.3f} 0 {origin_z:.3f}")
        collision_origin.set("rpy", "0 0 0")

        collision_geometry = ET.SubElement(collision, "geometry")
        collision_box = ET.SubElement(collision_geometry, "box")
        collision_box.set("size", f"{length:.3f} {width:.3f} {height:.3f}")

        # Add inertial properties if available
        physics = vehicle_params.get("physics", {})
        if physics.get("mass"):
            inertial = ET.SubElement(base_link, "inertial")

            # Mass
            mass_elem = ET.SubElement(inertial, "mass")
            mass_elem.set("value", str(physics["mass"]))

            # Origin at center of mass
            com = physics.get("center_of_mass", {"x": 0, "y": 0, "z": 0})
            inertial_origin = ET.SubElement(inertial, "origin")
            inertial_origin.set("xyz", f"{com['x']:.3f} {com['y']:.3f} {com['z']:.3f}")
            inertial_origin.set("rpy", "0 0 0")

            # Inertia tensor (simplified box approximation)
            inertia = ET.SubElement(inertial, "inertia")
            m = physics["mass"]
            ixx = (m / 12.0) * (width**2 + height**2)
            iyy = (m / 12.0) * (length**2 + height**2)
            izz = (m / 12.0) * (length**2 + width**2)

            inertia.set("ixx", f"{ixx:.3f}")
            inertia.set("ixy", "0")
            inertia.set("ixz", "0")
            inertia.set("iyy", f"{iyy:.3f}")
            inertia.set("iyz", "0")
            inertia.set("izz", f"{izz:.3f}")

    def _add_sensor_kit_link(self, robot: ET.Element):
        """
        Add sensor_kit_base_link to URDF.

        Args:
            robot: Root robot element
        """
        # Create sensor_kit_base_link
        sensor_kit_link = ET.SubElement(robot, "link", name="sensor_kit_base_link")

        # Add a small visual marker
        visual = ET.SubElement(sensor_kit_link, "visual")
        visual_origin = ET.SubElement(visual, "origin")
        visual_origin.set("xyz", "0 0 0")
        visual_origin.set("rpy", "0 0 0")

        geometry = ET.SubElement(visual, "geometry")
        box = ET.SubElement(geometry, "box")
        box.set("size", "0.1 0.1 0.1")

        # Create joint connecting base_link to sensor_kit_base_link
        joint = ET.SubElement(
            robot, "joint", name="sensor_kit_base_joint", type="fixed"
        )
        parent = ET.SubElement(joint, "parent", link="base_link")
        child = ET.SubElement(joint, "child", link="sensor_kit_base_link")

        # Default sensor kit position (on vehicle roof)
        # This should match the calibration YAML files
        joint_origin = ET.SubElement(joint, "origin")
        joint_origin.set("xyz", "0 0 1.7")  # 1.7m above base_link
        joint_origin.set("rpy", "0 0 0")

    def _add_sensor_link(
        self, robot: ET.Element, sensor_name: str, sensor_config: Dict[str, Any]
    ):
        """
        Add individual sensor link to URDF.

        Args:
            robot: Root robot element
            sensor_name: Name of the sensor
            sensor_config: Sensor configuration dictionary
        """
        # Create sensor link
        link_name = f"{sensor_name}_link"
        sensor_link = ET.SubElement(robot, "link", name=link_name)

        # Add visual representation based on sensor type
        visual = ET.SubElement(sensor_link, "visual")
        visual_origin = ET.SubElement(visual, "origin")
        visual_origin.set("xyz", "0 0 0")
        visual_origin.set("rpy", "0 0 0")

        geometry = ET.SubElement(visual, "geometry")

        # Choose geometry based on sensor type
        sensor_type = sensor_config.get("sensor_type", "unknown")
        if sensor_type == "camera":
            box = ET.SubElement(geometry, "box")
            box.set("size", "0.05 0.05 0.05")
        elif sensor_type == "lidar":
            cylinder = ET.SubElement(geometry, "cylinder")
            cylinder.set("radius", "0.05")
            cylinder.set("length", "0.1")
        elif sensor_type in ["imu", "gnss"]:
            box = ET.SubElement(geometry, "box")
            box.set("size", "0.02 0.02 0.02")
        else:
            sphere = ET.SubElement(geometry, "sphere")
            sphere.set("radius", "0.02")

        # Create joint connecting sensor to sensor_kit_base_link
        joint_name = f"{sensor_name}_joint"
        joint = ET.SubElement(robot, "joint", name=joint_name, type="fixed")
        parent = ET.SubElement(joint, "parent", link="sensor_kit_base_link")
        child = ET.SubElement(joint, "child", link=link_name)

        # Set joint transform from sensor configuration
        transform = sensor_config.get("transform", {})
        joint_origin = ET.SubElement(joint, "origin")

        # Position
        x = transform.get("x", 0.0)
        y = transform.get("y", 0.0)
        z = transform.get("z", 0.0)
        joint_origin.set("xyz", f"{x:.3f} {y:.3f} {z:.3f}")

        # Orientation
        roll = transform.get("roll", 0.0)
        pitch = transform.get("pitch", 0.0)
        yaw = transform.get("yaw", 0.0)
        joint_origin.set("rpy", f"{roll:.3f} {pitch:.3f} {yaw:.3f}")

    def _prettify_xml(self, elem: ET.Element) -> str:
        """
        Return a pretty-printed XML string for the Element.

        Args:
            elem: XML element to prettify

        Returns:
            Pretty-printed XML string
        """
        rough_string = ET.tostring(elem, encoding="unicode")
        reparsed = minidom.parseString(rough_string)

        # Get pretty XML and remove extra blank lines
        pretty_xml = reparsed.toprettyxml(indent="  ")

        # Remove the XML declaration and empty lines
        lines = pretty_xml.split("\n")
        # Skip the XML declaration line
        if lines[0].startswith("<?xml"):
            lines = lines[1:]
        # Remove empty lines
        lines = [line for line in lines if line.strip()]

        return "\n".join(lines)

    def generate_sensor_kit_xacro(
        self, sensor_params: Dict[str, Any], namespace: str = ""
    ) -> str:
        """
        Generate sensor kit xacro file content.

        Args:
            sensor_params: Sensor parameters from SensorParameterExtractor
            namespace: Optional namespace prefix

        Returns:
            Xacro XML as string
        """
        # This could be extended to generate full xacro with macros
        # For now, we'll use the simpler URDF approach
        # Xacro generation would be similar but with macro definitions
        pass

    def validate_urdf(self, urdf_string: str) -> bool:
        """
        Basic validation of generated URDF.

        Args:
            urdf_string: URDF XML string to validate

        Returns:
            True if valid, False otherwise
        """
        try:
            # Parse the XML
            root = ET.fromstring(urdf_string)

            # Check root element is 'robot'
            if root.tag != "robot":
                return False

            # Check for required elements
            links = root.findall(".//link")
            if not links:
                return False

            # Check for base_link
            base_link_found = any(link.get("name") == "base_link" for link in links)
            if not base_link_found:
                return False

            # Basic structure validation passed
            return True

        except ET.ParseError:
            return False


class DynamicConfigGenerator:
    """High-level generator that combines vehicle, sensor, and URDF generation"""

    def __init__(self, world: "carla.World", vehicle: "carla.Vehicle"):
        """
        Initialize the configuration generator.

        Args:
            world: CARLA world instance
            vehicle: CARLA vehicle actor
        """
        # Import here to avoid circular imports
        from .parameter_extractor import (
            VehicleParameterExtractor,
            SensorParameterExtractor,
        )

        self.vehicle_extractor = VehicleParameterExtractor(vehicle)
        self.sensor_extractor = SensorParameterExtractor(world, vehicle)
        self.urdf_generator = DynamicURDFGenerator()

    def generate_vehicle_parameters(self) -> Dict[str, Any]:
        """
        Generate complete vehicle parameter set.

        Returns:
            Dictionary with all vehicle parameters
        """
        return self.vehicle_extractor.extract_all_parameters()

    def generate_sensor_parameters(self) -> Dict[str, Any]:
        """
        Generate sensor configuration parameters.

        Returns:
            Dictionary mapping sensor names to configurations
        """
        return self.sensor_extractor.extract_all_sensor_parameters()

    def generate_urdf_string(self) -> str:
        """
        Generate URDF robot description as string.

        Returns:
            URDF XML string
        """
        vehicle_params = self.generate_vehicle_parameters()
        sensor_params = self.generate_sensor_parameters()

        return self.urdf_generator.generate_urdf_string(vehicle_params, sensor_params)

    def generate_all_configurations(self) -> Dict[str, Any]:
        """
        Generate all configurations in one call.

        Returns:
            Dictionary containing:
            - vehicle_parameters: All vehicle parameters
            - sensor_parameters: All sensor parameters
            - robot_description: URDF string
        """
        vehicle_params = self.generate_vehicle_parameters()
        sensor_params = self.generate_sensor_parameters()
        urdf_string = self.urdf_generator.generate_urdf_string(
            vehicle_params, sensor_params
        )

        return {
            "vehicle_parameters": vehicle_params,
            "sensor_parameters": sensor_params,
            "robot_description": urdf_string,
        }
