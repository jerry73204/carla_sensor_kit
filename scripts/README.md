# Scripts Directory

This directory contains utility scripts for CARLA-Autoware integration.

## Main Scripts

### spawn_vehicle.py
Simple wrapper to spawn vehicles with sensors using the ROS 2 node.

```bash
python3 spawn_vehicle.py --host localhost --port 2000 --vehicle-name hero
```

### run_carla_autoware.py
Complete automation script that:
- Starts CARLA server
- Spawns vehicles with sensors
- Launches Autoware sensor kit
- Monitors system status

```bash
python3 run_carla_autoware.py
```

### sensor_config_loader.py
Python module for loading sensor configurations from YAML files and converting
between ROS and CARLA coordinate systems.

## Development Scripts

### carla.rs
Rust script for starting CARLA server (internal testing).

### test_sensor_placement.py
Test script to verify sensor placement configurations.

### simple_spawn_integrated.py
Legacy spawner script that directly uses CARLA Python API.

## ROS 2 Integration

The main vehicle spawning functionality is now available as a ROS 2 node in the
`carla_sensor_kit_utils` package. Use the node for better integration:

```bash
ros2 launch carla_sensor_kit_utils vehicle_spawner.launch.py
```

## Testing

Run tests with:
```bash
./run_tests.sh
```