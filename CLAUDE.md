# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a ROS2 sensor kit launch package for integrating CARLA simulator with Autoware. It provides launch files and configurations for various sensors including LiDAR, cameras, IMU, and GNSS.

## Build Commands

```bash
# Build all packages
colcon build

# Build specific package
colcon build --packages-select carla_sensor_kit_launch
colcon build --packages-select carla_sensor_kit_description

# Clean build
rm -rf build/ install/ log/
colcon build
```

## Testing

```bash
# Run linting tests
colcon test
colcon test-result --verbose
```

## Architecture

### Package Structure
- **carla_sensor_kit_launch**: Main launch package containing sensor launch configurations
  - Launch files orchestrate sensor drivers (LiDAR, camera, IMU, GNSS)
  - Config files contain ROS parameters for nodes
  
- **carla_sensor_kit_description**: URDF/xacro descriptions for sensor mounting
  - Defines sensor positions and transformations
  - Contains calibration configurations

### Key Launch Files
- `sensing.launch.xml`: Main entry point that includes all sensor launches
- Individual sensor launches: `lidar.launch.xml`, `camera.launch.xml`, `imu.launch.xml`, `gnss.launch.xml`
- `pointcloud_preprocessor.launch.py`: Python launch file for pointcloud processing pipeline

### Dependencies
- Autoware packages: `autoware_gnss_poser`, `autoware_pointcloud_preprocessor`, `autoware_vehicle_velocity_converter`
- Sensor drivers: `ublox_gps`, `usb_cam`
- Uses `ament_cmake` build system with `ament_cmake_auto` for dependency management