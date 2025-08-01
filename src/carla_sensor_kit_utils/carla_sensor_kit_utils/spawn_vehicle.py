#!/usr/bin/env python3
"""
Dynamic Vehicle Spawner for CARLA

Spawns vehicles with sensors in CARLA based on a YAML configuration file.
"""

import argparse
import carla
import random
import sys
import time
import yaml
import os

# No additional package imports needed


class VehicleSpawner:
    """Spawns vehicles with sensors in CARLA."""

    def __init__(self, host="localhost", port=2000):
        """Initialize the spawner with CARLA connection."""
        try:
            self.client = carla.Client(host, port)
            self.client.set_timeout(10.0)
            self.world = self.client.get_world()
            print(f"Connected to CARLA at {host}:{port}")
        except Exception as e:
            print(f"Failed to connect to CARLA: {e}")
            sys.exit(1)

        self.vehicle = None
        self.sensors = []
        self.config = None

    def load_config(self, config_file):
        """Load configuration from a YAML file."""
        try:
            with open(config_file, "r") as f:
                self.config = yaml.safe_load(f)
                print(f"Loaded configuration from {config_file}")
                return True
        except Exception as e:
            print(f"Failed to load config file: {e}")
            return False

    def spawn_vehicle(
        self,
        vehicle_type=None,
        role_name=None,
        spawn_point_index=None,
    ):
        """Spawn a vehicle at the specified spawn point."""
        # Use config values if available and not overridden by arguments
        if self.config and "vehicle" in self.config:
            vehicle_config = self.config["vehicle"]
            vehicle_type = vehicle_type or vehicle_config.get(
                "blueprint", "vehicle.lincoln.mkz"
            )
            role_name = role_name or vehicle_config.get("role_name", "ego_vehicle")
            spawn_point_index = (
                spawn_point_index
                if spawn_point_index is not None
                else vehicle_config.get("spawn_point", -1)
            )
        else:
            # Use defaults if no config
            vehicle_type = vehicle_type or "vehicle.lincoln.mkz"
            role_name = role_name or "ego_vehicle"
            spawn_point_index = (
                spawn_point_index if spawn_point_index is not None else -1
            )

        blueprint_library = self.world.get_blueprint_library()

        # Get vehicle blueprint
        try:
            vehicle_bp = blueprint_library.find(vehicle_type)
        except:
            print(f"Vehicle type '{vehicle_type}' not found. Available vehicles:")
            for bp in blueprint_library.filter("vehicle.*"):
                print(f"  {bp.id}")
            return False

        # Set role name
        vehicle_bp.set_attribute("role_name", role_name)

        # Get spawn point
        spawn_points = self.world.get_map().get_spawn_points()
        if not spawn_points:
            print("No spawn points available")
            return False

        if spawn_point_index < 0 or spawn_point_index >= len(spawn_points):
            spawn_point = random.choice(spawn_points)
            print(f"Using random spawn point (out of {len(spawn_points)} available)")
        else:
            spawn_point = spawn_points[spawn_point_index]
            print(f"Using spawn point {spawn_point_index}")

        # Spawn the vehicle
        try:
            self.vehicle = self.world.spawn_actor(vehicle_bp, spawn_point)
            print(f"Spawned {vehicle_type} with role_name '{role_name}'")
            print(f"Vehicle ID: {self.vehicle.id}")
            return True
        except Exception as e:
            print(f"Failed to spawn vehicle: {e}")
            return False

    def attach_sensors(self):
        """Attach sensors to the vehicle based on configuration file."""
        if not self.vehicle:
            print("No vehicle spawned")
            return False

        if not self.config or "sensors" not in self.config:
            print("No sensors defined in configuration file")
            return False

        blueprint_library = self.world.get_blueprint_library()
        sensor_specs = self.config["sensors"]
        print(f"Attaching {len(sensor_specs)} sensors from configuration file")

        for spec in sensor_specs:
            try:
                sensor_bp = blueprint_library.find(spec["type"])
                sensor_bp.set_attribute("role_name", spec["name"])

                # Set additional attributes
                for attr, value in spec.get("attributes", {}).items():
                    sensor_bp.set_attribute(attr, str(value))

                # Create transform from config
                transform_data = spec.get(
                    "transform", {"location": [0, 0, 0], "rotation": [0, 0, 0]}
                )
                location = transform_data.get("location", [0, 0, 0])
                rotation = transform_data.get("rotation", [0, 0, 0])

                transform = carla.Transform(
                    carla.Location(x=location[0], y=location[1], z=location[2]),
                    carla.Rotation(
                        pitch=rotation[0], yaw=rotation[1], roll=rotation[2]
                    ),
                )

                # Spawn sensor
                sensor = self.world.spawn_actor(
                    sensor_bp, transform, attach_to=self.vehicle
                )
                self.sensors.append(sensor)
                print(f"  Attached {spec['name']} ({spec['type']})")

            except Exception as e:
                print(f"  Failed to attach {spec['name']}: {e}")

        print(f"Attached {len(self.sensors)} sensors")
        return True

    def cleanup(self):
        """Clean up spawned actors."""
        # Destroy sensors first
        for sensor in self.sensors:
            if sensor.is_alive:
                sensor.destroy()

        # Destroy vehicle
        if self.vehicle and self.vehicle.is_alive:
            self.vehicle.destroy()

        print("Cleaned up spawned actors")
    
    def cleanup_by_role_name(self, role_name):
        """Find and clean up vehicle and its sensors by role name."""
        actors = self.world.get_actors()
        vehicles = actors.filter("vehicle.*")
        sensors = actors.filter("sensor.*")
        
        found_vehicle = None
        for vehicle in vehicles:
            if vehicle.attributes.get("role_name") == role_name:
                found_vehicle = vehicle
                break
        
        if not found_vehicle:
            print(f"No vehicle found with role_name '{role_name}'")
            return False
        
        # Find and destroy sensors attached to this vehicle
        destroyed_sensors = 0
        for sensor in sensors:
            if sensor.parent and sensor.parent.id == found_vehicle.id:
                sensor.destroy()
                destroyed_sensors += 1
        
        # Destroy the vehicle
        vehicle_type = found_vehicle.type_id
        found_vehicle.destroy()
        
        print(f"Cleaned up vehicle '{role_name}' ({vehicle_type}) and {destroyed_sensors} sensors")
        return True


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="Spawn a vehicle with sensors in CARLA",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Spawn using default config file
  %(prog)s --port 3000

  # Spawn with custom config file
  %(prog)s --config my_config.yaml --port 3000
  
  # Override config file settings
  %(prog)s --vehicle-type vehicle.audi.tt --role-name my_car

  # Clean up a spawned vehicle
  %(prog)s --cleanup --role-name my_car
  
  # Clean up default vehicle (ego_vehicle)
  %(prog)s --cleanup

  # List available vehicles
  %(prog)s --list-vehicles
        """,
    )

    parser.add_argument("--host", default="localhost", help="CARLA server host")
    parser.add_argument("--port", type=int, default=2000, help="CARLA server port")

    # Config file argument
    # Try to find config in multiple locations
    possible_configs = [
        # Development location
        os.path.join(
            os.path.dirname(os.path.dirname(__file__)),
            "config",
            "spawn_vehicle_config.yaml",
        ),
        # Installed location
        os.path.join(
            os.path.dirname(os.path.dirname(os.path.dirname(__file__))),
            "share",
            "carla_sensor_kit_utils",
            "config",
            "spawn_vehicle_config.yaml",
        ),
    ]

    default_config = None
    for config_path in possible_configs:
        if os.path.exists(config_path):
            default_config = config_path
            break

    if not default_config:
        default_config = possible_configs[0]  # Use first as fallback
    parser.add_argument(
        "--config",
        "-c",
        default=default_config,
        help="Configuration file (YAML format)",
    )

    # Vehicle arguments (override config)
    parser.add_argument(
        "--vehicle-type", help="Vehicle blueprint ID (overrides config)"
    )
    parser.add_argument("--role-name", help="Vehicle role name (overrides config)")
    parser.add_argument(
        "--spawn-point", type=int, help="Spawn point index (overrides config)"
    )

    parser.add_argument(
        "--list-vehicles",
        action="store_true",
        help="List available vehicle types and exit",
    )
    
    parser.add_argument(
        "--cleanup",
        action="store_true",
        help="Remove spawned vehicle with the specified role name",
    )

    args = parser.parse_args()

    # Create spawner
    spawner = VehicleSpawner(args.host, args.port)

    # List vehicles if requested
    if args.list_vehicles:
        print("Available vehicle types:")
        blueprint_library = spawner.world.get_blueprint_library()
        for bp in sorted(blueprint_library.filter("vehicle.*"), key=lambda x: x.id):
            print(f"  {bp.id}")
        return 0

    # Handle cleanup mode
    if args.cleanup:
        # For cleanup, we need a role name
        if args.role_name:
            role_name = args.role_name
        elif os.path.exists(args.config):
            # Try to get role name from config
            if spawner.load_config(args.config):
                role_name = spawner.config.get("vehicle", {}).get("role_name", "ego_vehicle")
            else:
                print(f"Error: Failed to load config file {args.config}")
                return 1
        else:
            role_name = "ego_vehicle"  # Default
        
        print(f"Cleaning up vehicle with role_name '{role_name}'...")
        if spawner.cleanup_by_role_name(role_name):
            return 0
        else:
            return 1
    
    # Normal spawn mode
    # Load configuration file
    if os.path.exists(args.config):
        if not spawner.load_config(args.config):
            print(f"Error: Failed to load config file {args.config}")
            return 1
    else:
        print(f"Error: Config file {args.config} not found")
        print("Please provide a valid configuration file with --config option")
        return 1

    # Spawn new vehicle
    if not spawner.spawn_vehicle(args.vehicle_type, args.role_name, args.spawn_point):
        return 1

    # Attach sensors
    spawner.attach_sensors()

    # Get the actual role name used
    role_name = args.role_name or (
        spawner.config.get("vehicle", {}).get("role_name", "ego_vehicle")
        if spawner.config
        else "ego_vehicle"
    )

    print(f"\nVehicle '{role_name}' is ready!")
    print(f"To remove it later, run: spawn_vehicle --cleanup --role-name {role_name}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
