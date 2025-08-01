#!/usr/bin/env python3
"""
CARLA Package Generator

Generates ROS2 packages for CARLA sensor kit and vehicle configurations
by extracting parameters from a running CARLA simulation.
"""

import argparse
import os
import sys
import shutil
from pathlib import Path
import yaml
import json
from typing import Dict, Any, Optional, List
from jinja2 import Environment, FileSystemLoader, Template

try:
    import carla
except ImportError:
    print(
        "Warning: CARLA Python API not found. Use --no-carla mode with parameter file."
    )
    carla = None

from .parameter_extractor import VehicleParameterExtractor, SensorParameterExtractor


class PackageGenerator:
    """Generates ROS2 packages from CARLA parameters using Jinja2 templates."""

    def __init__(self, template_dir: Optional[Path] = None):
        """Initialize the package generator.

        Args:
            template_dir: Custom template directory path
        """
        if template_dir is None:
            # Default template directory
            self.template_dir = Path(__file__).parent / "templates"
        else:
            self.template_dir = Path(template_dir)

        if not self.template_dir.exists():
            raise ValueError(f"Template directory not found: {self.template_dir}")

        self.env = Environment(
            loader=FileSystemLoader(str(self.template_dir)),
            trim_blocks=True,
            lstrip_blocks=True,
        )

        # Available sensor presets
        self.sensor_presets = {
            "default": {
                "lidar": [
                    {
                        "name": "velodyne_top",
                        "type": "sensor.lidar.ray_cast",
                        "transform": {"location": [0, 0, 2.0], "rotation": [0, 0, 0]},
                        "channels": 32,
                        "range": 100.0,
                        "points_per_second": 600000,
                        "rotation_frequency": 10.0,
                        "upper_fov": 10.0,
                        "lower_fov": -30.0,
                    }
                ],
                "camera": [
                    {
                        "name": "camera_front",
                        "type": "sensor.camera.rgb",
                        "transform": {"location": [0.8, 0, 1.5], "rotation": [0, 0, 0]},
                        "image_size_x": 1920,
                        "image_size_y": 1080,
                        "fov": 90,
                    }
                ],
                "imu": [
                    {
                        "name": "imu",
                        "type": "sensor.other.imu",
                        "transform": {"location": [0, 0, 0], "rotation": [0, 0, 0]},
                    }
                ],
                "gnss": [
                    {
                        "name": "gnss",
                        "type": "sensor.other.gnss",
                        "transform": {"location": [0, 0, 0], "rotation": [0, 0, 0]},
                    }
                ],
            },
            "minimal": {
                "lidar": [
                    {
                        "name": "velodyne_top",
                        "type": "sensor.lidar.ray_cast",
                        "transform": {"location": [0, 0, 2.0], "rotation": [0, 0, 0]},
                        "channels": 16,
                        "range": 80.0,
                        "points_per_second": 300000,
                        "rotation_frequency": 10.0,
                        "upper_fov": 10.0,
                        "lower_fov": -30.0,
                    }
                ],
                "gnss": [
                    {
                        "name": "gnss",
                        "type": "sensor.other.gnss",
                        "transform": {"location": [0, 0, 0], "rotation": [0, 0, 0]},
                    }
                ],
            },
            "advanced": {
                "lidar": [
                    {
                        "name": "velodyne_top",
                        "type": "sensor.lidar.ray_cast",
                        "transform": {"location": [0, 0, 2.0], "rotation": [0, 0, 0]},
                        "channels": 64,
                        "range": 120.0,
                        "points_per_second": 1200000,
                        "rotation_frequency": 10.0,
                        "upper_fov": 10.0,
                        "lower_fov": -30.0,
                    },
                    {
                        "name": "velodyne_front",
                        "type": "sensor.lidar.ray_cast",
                        "transform": {
                            "location": [2.0, 0, 0.5],
                            "rotation": [-15, 0, 0],
                        },
                        "channels": 32,
                        "range": 80.0,
                        "points_per_second": 600000,
                        "rotation_frequency": 10.0,
                        "upper_fov": 10.0,
                        "lower_fov": -30.0,
                    },
                ],
                "camera": [
                    {
                        "name": "camera_front",
                        "type": "sensor.camera.rgb",
                        "transform": {"location": [0.8, 0, 1.5], "rotation": [0, 0, 0]},
                        "image_size_x": 1920,
                        "image_size_y": 1080,
                        "fov": 90,
                    },
                    {
                        "name": "camera_left",
                        "type": "sensor.camera.rgb",
                        "transform": {
                            "location": [0, -0.5, 1.5],
                            "rotation": [0, -45, 0],
                        },
                        "image_size_x": 1920,
                        "image_size_y": 1080,
                        "fov": 90,
                    },
                    {
                        "name": "camera_right",
                        "type": "sensor.camera.rgb",
                        "transform": {
                            "location": [0, 0.5, 1.5],
                            "rotation": [0, 45, 0],
                        },
                        "image_size_x": 1920,
                        "image_size_y": 1080,
                        "fov": 90,
                    },
                ],
                "radar": [
                    {
                        "name": "radar_front",
                        "type": "sensor.other.radar",
                        "transform": {"location": [2.0, 0, 0.5], "rotation": [0, 0, 0]},
                        "horizontal_fov": 30,
                        "vertical_fov": 30,
                        "range": 100,
                        "points_per_second": 1500,
                    }
                ],
                "imu": [
                    {
                        "name": "imu",
                        "type": "sensor.other.imu",
                        "transform": {"location": [0, 0, 0], "rotation": [0, 0, 0]},
                    }
                ],
                "gnss": [
                    {
                        "name": "gnss",
                        "type": "sensor.other.gnss",
                        "transform": {"location": [0, 0, 0], "rotation": [0, 0, 0]},
                    }
                ],
            },
        }

    def extract_parameters_from_carla(
        self, host: str, port: int, vehicle_role_name: str
    ) -> Dict[str, Any]:
        """Extract parameters from a running CARLA simulation.

        Args:
            host: CARLA server host
            port: CARLA server port
            vehicle_role_name: Role name of the vehicle to extract

        Returns:
            Dictionary containing vehicle and sensor parameters
        """
        if carla is None:
            raise RuntimeError("CARLA Python API not available")

        # Connect to CARLA
        client = carla.Client(host, port)
        client.set_timeout(10.0)
        world = client.get_world()

        # Find vehicle by role name
        actors = world.get_actors()
        vehicles = actors.filter("vehicle.*")

        target_vehicle = None
        for vehicle in vehicles:
            if vehicle.attributes.get("role_name") == vehicle_role_name:
                target_vehicle = vehicle
                break

        if target_vehicle is None:
            raise ValueError(f"No vehicle found with role_name '{vehicle_role_name}'")

        # Extract vehicle parameters
        vehicle_extractor = VehicleParameterExtractor(target_vehicle)
        vehicle_params = vehicle_extractor.extract_all_parameters()
        vehicle_params["role_name"] = vehicle_role_name

        # Extract sensor parameters
        sensor_extractor = SensorParameterExtractor(world, target_vehicle)
        sensor_params = sensor_extractor.extract_all_sensor_parameters()
        
        # Reorganize sensors by type for templates
        sensors_by_type = {}
        for sensor_name, sensor_data in sensor_params.items():
            sensor_type = sensor_data.get('sensor_type', 'unknown')
            if sensor_type not in sensors_by_type:
                sensors_by_type[sensor_type] = []
            # Add the name to the sensor data
            sensor_data['name'] = sensor_name
            sensors_by_type[sensor_type].append(sensor_data)

        return {"vehicle": vehicle_params, "sensors": sensors_by_type}

    def generate_from_carla(
        self,
        host: str,
        port: int,
        vehicle_role_name: str,
        sensor_kit_name: str,
        vehicle_model_name: str,
        output_dir: Path,
    ) -> List[Path]:
        """Generate packages from CARLA simulation.

        Args:
            host: CARLA server host
            port: CARLA server port
            vehicle_role_name: Role name of the vehicle
            sensor_kit_name: Name for sensor kit packages
            vehicle_model_name: Name for vehicle packages
            output_dir: Output directory for generated packages

        Returns:
            List of generated package directories
        """
        params = self.extract_parameters_from_carla(host, port, vehicle_role_name)
        # Add CARLA connection parameters
        params['carla_host'] = host
        params['carla_port'] = port
        return self.generate_from_params(
            params, sensor_kit_name, vehicle_model_name, output_dir
        )

    def generate_from_params(
        self,
        params: Dict[str, Any],
        sensor_kit_name: str,
        vehicle_model_name: str,
        output_dir: Path,
    ) -> List[Path]:
        """Generate packages from parameter dictionary.

        Args:
            params: Dictionary containing vehicle and sensor parameters
            sensor_kit_name: Name for sensor kit packages
            vehicle_model_name: Name for vehicle packages
            output_dir: Output directory for generated packages

        Returns:
            List of generated package directories
        """
        output_dir = Path(output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)

        # Package names
        packages = [
            f"{sensor_kit_name}_description",
            f"{sensor_kit_name}_launch",
            f"{vehicle_model_name}_description",
            f"{vehicle_model_name}_launch",
        ]

        generated_paths = []

        for package_name in packages:
            package_path = output_dir / package_name
            self._generate_package(
                package_name, params, sensor_kit_name, vehicle_model_name, package_path
            )
            generated_paths.append(package_path)

        return generated_paths

    def _generate_package(
        self,
        package_name: str,
        params: Dict[str, Any],
        sensor_kit_name: str,
        vehicle_model_name: str,
        output_path: Path,
    ) -> None:
        """Generate a single package.

        Args:
            package_name: Name of the package to generate
            params: Parameter dictionary
            sensor_kit_name: Sensor kit name
            vehicle_model_name: Vehicle model name
            output_path: Output path for the package
        """
        # Determine template source directory
        if package_name.endswith("_description"):
            if sensor_kit_name in package_name:
                template_pkg = "carla_sensor_kit_description"
            else:
                template_pkg = "carla_vehicle_description"
        else:  # _launch
            if sensor_kit_name in package_name:
                template_pkg = "carla_sensor_kit_launch"
            else:
                template_pkg = "carla_vehicle_launch"

        template_path = self.template_dir / template_pkg

        if not template_path.exists():
            raise ValueError(f"Template package not found: {template_path}")

        # Create output directory
        output_path.mkdir(parents=True, exist_ok=True)

        # Process all files in template directory
        for root, dirs, files in os.walk(template_path):
            # Skip COLCON_IGNORE files
            if "COLCON_IGNORE" in files:
                continue

            # Calculate relative path
            rel_path = Path(root).relative_to(template_path)
            dest_dir = output_path / rel_path
            dest_dir.mkdir(parents=True, exist_ok=True)

            # Process files
            for file in files:
                src_file = Path(root) / file

                # Prepare template context
                context = {
                    "package_name": package_name,
                    "sensor_kit_name": sensor_kit_name,
                    "vehicle_model_name": vehicle_model_name,
                    "vehicle": params.get("vehicle", {}),
                    "sensors": params.get("sensors", {}),
                    "carla_host": params.get("carla_host", "localhost"),
                    "carla_port": params.get("carla_port", 2000),
                }

                if file.endswith(".j2"):
                    # Process Jinja2 template
                    dest_file = dest_dir / file[:-3]  # Remove .j2 extension
                    template = self.env.get_template(
                        str(Path(template_pkg) / rel_path / file)
                    )
                    content = template.render(**context)
                    dest_file.write_text(content)
                else:
                    # Copy non-template files
                    dest_file = dest_dir / file
                    shutil.copy2(src_file, dest_file)

        print(f"Generated package: {package_name}")

    def list_presets(self) -> None:
        """List available sensor presets."""
        print("Available sensor presets:")
        for name, preset in self.sensor_presets.items():
            print(f"\n{name}:")
            for sensor_type, sensors in preset.items():
                if sensors:
                    print(f"  - {sensor_type}: {len(sensors)} sensor(s)")


def main():
    """Main entry point for the package generator."""
    parser = argparse.ArgumentParser(
        description="Generate ROS2 packages from CARLA parameters",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Generate from running CARLA simulation
  %(prog)s --host localhost --port 2000 --vehicle ego_vehicle
  
  # Generate with custom names
  %(prog)s --sensor-kit my_sensors --vehicle-model my_vehicle
  
  # Use sensor preset
  %(prog)s --sensor-preset minimal
  
  # Generate offline from parameters
  %(prog)s --no-carla --params saved_params.yaml
        """,
    )

    # CARLA connection arguments
    carla_group = parser.add_argument_group("CARLA connection")
    carla_group.add_argument(
        "--host", default="localhost", help="CARLA server host (default: localhost)"
    )
    carla_group.add_argument(
        "--port", type=int, default=2000, help="CARLA server port (default: 2000)"
    )
    carla_group.add_argument(
        "--vehicle",
        default="ego_vehicle",
        help="Vehicle role name in CARLA (default: ego_vehicle)",
    )

    # Package naming arguments
    naming_group = parser.add_argument_group("Package naming")
    naming_group.add_argument(
        "--sensor-kit",
        default="carla_sensor_kit",
        help="Name for sensor kit packages (default: carla_sensor_kit)",
    )
    naming_group.add_argument(
        "--vehicle-model",
        default="carla_vehicle",
        help="Name for vehicle packages (default: carla_vehicle)",
    )

    # Output options
    output_group = parser.add_argument_group("Output options")
    output_group.add_argument(
        "-o",
        "--output",
        default="./workspace",
        help="Output directory (default: ./workspace)",
    )
    output_group.add_argument(
        "--clean", action="store_true", help="Clean output directory before generating"
    )

    # Configuration options
    config_group = parser.add_argument_group("Configuration options")
    config_group.add_argument(
        "--sensor-preset",
        choices=["default", "minimal", "advanced"],
        help="Use predefined sensor configuration",
    )
    config_group.add_argument(
        "--params", help="Load/save parameters from/to YAML/JSON file"
    )
    config_group.add_argument(
        "--no-carla",
        action="store_true",
        help="Generate without CARLA connection (requires --params)",
    )

    # Other options
    parser.add_argument("--template-dir", help="Custom template directory")
    parser.add_argument(
        "--list-presets", action="store_true", help="List available presets and exit"
    )
    parser.add_argument(
        "-v", "--verbose", action="store_true", help="Enable verbose output"
    )

    args = parser.parse_args()

    # Create generator
    try:
        generator = PackageGenerator(args.template_dir)
    except Exception as e:
        print(f"Error initializing generator: {e}")
        return 1

    # List presets if requested
    if args.list_presets:
        generator.list_presets()
        return 0

    # Validate arguments
    if args.no_carla and not args.params:
        print("Error: --no-carla requires --params file")
        return 1

    # Clean output directory if requested
    output_dir = Path(args.output)
    if args.clean and output_dir.exists():
        print(f"Cleaning output directory: {output_dir}")
        shutil.rmtree(output_dir)

    try:
        # Get parameters
        if args.no_carla:
            # Load from file
            params_file = Path(args.params)
            if not params_file.exists():
                print(f"Error: Parameter file not found: {params_file}")
                return 1

            with open(params_file) as f:
                if params_file.suffix == ".json":
                    params = json.load(f)
                else:
                    params = yaml.safe_load(f)

            print(f"Loaded parameters from {params_file}")
        else:
            # Extract from CARLA
            if args.verbose:
                print(f"Connecting to CARLA at {args.host}:{args.port}")

            params = generator.extract_parameters_from_carla(
                args.host, args.port, args.vehicle
            )

            # Save parameters if requested
            if args.params:
                params_file = Path(args.params)
                with open(params_file, "w") as f:
                    yaml.dump(params, f, default_flow_style=False)
                print(f"Saved parameters to {params_file}")

        # Apply sensor preset if specified
        if args.sensor_preset:
            if args.sensor_preset in generator.sensor_presets:
                params["sensors"] = generator.sensor_presets[args.sensor_preset]
                print(f"Applied sensor preset: {args.sensor_preset}")
            else:
                print(f"Warning: Unknown preset '{args.sensor_preset}'")

        # Generate packages
        print(f"\nGenerating packages in {output_dir}")
        generated = generator.generate_from_params(
            params, args.sensor_kit, args.vehicle_model, output_dir
        )

        print(f"\nSuccessfully generated {len(generated)} packages:")
        for pkg_path in generated:
            print(f"  - {pkg_path.name}")

        print(f"\nTo build the packages:")
        print(f"  cd {output_dir}")
        print(f"  colcon build")
        print(f"  source install/setup.bash")

        return 0

    except Exception as e:
        print(f"Error: {e}")
        if args.verbose:
            import traceback

            traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())
