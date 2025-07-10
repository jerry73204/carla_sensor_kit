# CARLA Sensor Kit Launch

A ROS 2 sensor kit package for integrating CARLA simulator with Autoware. This package provides launch files and configurations for LiDAR, IMU, and GNSS sensors, enabling seamless topic remapping from CARLA to Autoware's sensor processing pipeline.

## Features

- Topic remapping from CARLA sensor topics to Autoware format
- Support for multiple LiDAR sensors (top, left, right, rear)
- IMU data processing with bias estimation
- GNSS positioning with pose conversion
- Pointcloud preprocessing and concatenation
- ROS 2 node for vehicle spawning with proper sensor placement
- No physical hardware dependencies

## Prerequisites

- ROS 2 Humble
- CARLA 0.10.0 or later
- Python 3.8+
- Autoware packages (installed via apt or built from source)

## Installation

```bash
# Clone this repository
cd ~/ros2_ws/src
git clone <repository_url> carla_sensor_kit_launch

# Build the packages
cd ~/ros2_ws
colcon build --packages-select carla_sensor_kit_launch carla_sensor_kit_description carla_sensor_kit_utils
```

## Usage

### Method 1: Step-by-Step Setup

1. **Start CARLA server**
   ```bash
   # Download CARLA 0.10.0 from:
   # https://github.com/carla-simulator/carla/releases
   
   # Extract and run CARLA server
   cd CARLA_0.10.0
   ./CarlaUE4.sh
   ```

2. **Spawn vehicle with sensors**
   ```bash
   # Option A: Using ROS 2 launch file
   ros2 launch carla_sensor_kit_utils vehicle_spawner.launch.py
   
   # Option B: Using convenience script
   python3 scripts/spawn_vehicle.py --vehicle-name hero
   ```

3. **Launch Autoware sensor kit**
   ```bash
   # Terminal 1: Launch sensor kit for topic remapping
   export VEHICLE_ID=hero
   ros2 launch carla_sensor_kit_launch sensing.launch.xml
   
   # Terminal 2 (Optional): Launch full Autoware stack
   # Follow Autoware documentation for complete setup
   ```

### Method 2: Automated Workflow

Use the all-in-one script that handles CARLA server, vehicle spawning, and sensor kit launch:

```bash
python3 scripts/run_carla_autoware.py
```

### Vehicle Spawner Options

The vehicle spawner supports various configuration options:

```bash
# Spawn with custom vehicle type
ros2 launch carla_sensor_kit_utils vehicle_spawner.launch.py \
  vehicle_type:=vehicle.audi.a2 \
  vehicle_role_name:=ego_vehicle

# Spawn without autopilot
ros2 launch carla_sensor_kit_utils vehicle_spawner.launch.py \
  autopilot:=False

# Spawn at specific location
ros2 launch carla_sensor_kit_utils vehicle_spawner.launch.py \
  spawn_point_index:=10
```

## Topic Mapping

The sensor kit creates the following topic mappings:

| CARLA Topic | Autoware Topic |
|-------------|----------------|
| `/carla/hero/lidar_top` | `/lidar/top/pointcloud_raw` |
| `/carla/hero/lidar_left` | `/lidar/left/pointcloud_raw` |
| `/carla/hero/lidar_right` | `/lidar/right/pointcloud_raw` |
| `/carla/hero/lidar_rear` | `/lidar/rear/pointcloud_raw` |
| `/carla/hero/imu` | `/imu/tamagawa/imu_raw` |
| `/carla/hero/gnss` | `/gnss/ublox/nav_sat_fix` |

## Packages Overview

### carla_sensor_kit_launch
Main launch package containing sensor topic remapping configurations and launch files.

### carla_sensor_kit_description
URDF/xacro descriptions and calibration files defining sensor positions and transformations.

### carla_sensor_kit_utils
Python utilities including the ROS 2 vehicle spawner node that reads sensor configurations and spawns vehicles in CARLA.

## Configuration

Sensor positions and calibration parameters are defined in:
- `carla_sensor_kit_description/config/sensor_kit_calibration.yaml`
- `carla_sensor_kit_launch/config/concatenate_and_time_sync_node.param.yaml`

## Developer Guide

### Package Structure

- **carla_sensor_kit_launch**: Main launch package with sensor configurations
- **carla_sensor_kit_description**: URDF/xacro sensor descriptions and calibration files
- **scripts/**: Python utilities for CARLA integration and testing

### Key Launch Files

- `sensing.launch.xml`: Main entry point for all sensors
- `lidar.launch.xml`: LiDAR topic remapping and preprocessing
- `imu.launch.xml`: IMU topic remapping and correction
- `gnss.launch.xml`: GNSS topic remapping and pose conversion

### Testing

```bash
# Run integration tests
colcon test --packages-select carla_sensor_kit_launch
colcon test-result --verbose

# Run custom test script
./run_tests.sh
```

### Extending the Sensor Kit

1. Add new sensor configurations to `sensor_kit_calibration.yaml`
2. Create corresponding launch files with topic remapping
3. Update the main `sensing.launch.xml` to include new sensors
4. Test with CARLA simulation

## Dependencies

- ROS 2 Humble
- Autoware packages (autoware_gnss_poser, autoware_pointcloud_preprocessor, etc.)
- CARLA 0.10.0 simulator
- Python 3.8+

## License

This project follows the same license as the Autoware project.