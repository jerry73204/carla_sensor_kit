# CARLA Vehicle Spawner Script Design

## Overview

This document describes the design of a Python script that spawns a vehicle with a complete sensor suite in CARLA simulator. The script prepares the simulation environment for use with the CARLA sensor kit launch package by creating properly named actors that can be remapped to Autoware topics.

## Purpose

The spawner script serves as the initialization step in the CARLA-Autoware integration workflow:

1. **User runs spawner script** → Vehicle with sensors created in CARLA
2. **User launches sensor kit** → Topics remapped from CARLA to Autoware
3. **Autoware receives data** → Sensor data flows through standard interfaces

## Script Requirements

### Functional Requirements

1. **Connection Management**
   - Connect to CARLA server (default: localhost:2000)
   - Handle connection timeouts gracefully
   - Verify server is responsive

2. **Vehicle Spawning**
   - Accept vehicle name as user input
   - Spawn vehicle at a safe location
   - Use vehicle name as actor name for consistency

3. **Sensor Attachment**
   - Attach standard Autoware sensor suite:
     - LiDAR (top-mounted)
     - Front camera
     - IMU
     - GNSS
   - Use predictable naming convention for sensors

4. **ROS Bridge Configuration**
   - Enable ROS bridge functionality for spawned actors
   - Ensure topics are published correctly

### Non-Functional Requirements

1. **Simplicity**
   - Single-file script with minimal dependencies
   - Clear command-line interface
   - No complex configuration files

2. **Reliability**
   - Graceful error handling
   - Clear error messages
   - Cleanup on script termination

3. **Compatibility**
   - Work with CARLA 0.10.0
   - Match expected topic names for sensor kit

## Design Details

### Command-Line Interface

```bash
python spawn_vehicle.py --name <vehicle_name> [options]
```

**Required Arguments:**
- `--name`: Name for the vehicle actor (e.g., "ego_vehicle")

**Optional Arguments:**
- `--host`: CARLA server host (default: "localhost")
- `--port`: CARLA server port (default: 2000)
- `--vehicle-filter`: Blueprint filter for vehicle type (default: "vehicle.tesla.model3")
- `--spawn-point`: Index of spawn point to use (default: random)

### Actor Naming Convention

To ensure compatibility with the sensor kit launch files, actors will be named as follows:

```
Vehicle: <user_provided_name>
LiDAR:   <user_provided_name>_lidar
Camera:  <user_provided_name>_camera
IMU:     <user_provided_name>_imu
GNSS:    <user_provided_name>_gnss
```

Example with `--name ego_vehicle`:
- Vehicle: `ego_vehicle`
- Topics:
  - `/carla/ego_vehicle/vehicle_status`
  - `/carla/ego_vehicle_lidar/point_cloud`
  - `/carla/ego_vehicle_camera/image`
  - `/carla/ego_vehicle_imu/imu`
  - `/carla/ego_vehicle_gnss/gnss`

### Sensor Specifications

#### LiDAR Configuration
- **Blueprint**: `sensor.lidar.ray_cast`
- **Position**: Top center of vehicle (x=0, y=0, z=2.5m)
- **Parameters**:
  - Channels: 64
  - Range: 100m
  - Rotation frequency: 20Hz
  - Points per second: 1,300,000
  - Upper FOV: +15°
  - Lower FOV: -25°

#### Camera Configuration
- **Blueprint**: `sensor.camera.rgb`
- **Position**: Front windshield (x=1.5m, y=0, z=1.5m)
- **Rotation**: Pitch -15° (looking slightly down)
- **Parameters**:
  - Resolution: 1920x1080
  - FOV: 90°
  - Sensor tick: 0.033s (30 FPS)

#### IMU Configuration
- **Blueprint**: `sensor.other.imu`
- **Position**: Vehicle center (x=0, y=0, z=0)
- **Parameters**:
  - Noise stddev (accel): 0.01 m/s²
  - Noise stddev (gyro): 0.01 rad/s
  - Sensor tick: 0.01s (100 Hz)

#### GNSS Configuration
- **Blueprint**: `sensor.other.gnss`
- **Position**: Vehicle center (x=0, y=0, z=0)
- **Parameters**:
  - Altitude noise stddev: 0.1m
  - Latitude noise stddev: 0.000005°
  - Longitude noise stddev: 0.000005°
  - Sensor tick: 0.1s (10 Hz)

### Script Flow

```
1. Parse command-line arguments
2. Connect to CARLA server
   └─ Handle connection errors
3. Get world and blueprint library
4. Find vehicle blueprint
   └─ Validate blueprint exists
5. Get spawn points from map
6. Select spawn point (random or specified)
7. Spawn vehicle with user-provided name
   └─ Retry if spawn fails
8. For each sensor type:
   a. Get sensor blueprint
   b. Configure sensor attributes
   c. Create attachment transform
   d. Spawn and attach sensor with naming convention
   └─ Handle attachment errors
9. Print summary of created actors
10. Keep script running until Ctrl+C
11. Cleanup: destroy all spawned actors
```

### Error Handling

1. **Connection Errors**
   - Clear message if CARLA server not reachable
   - Suggest starting CARLA first

2. **Spawn Failures**
   - Retry with different spawn point
   - Maximum 3 retry attempts
   - Clear error if all attempts fail

3. **Sensor Attachment Errors**
   - Continue with remaining sensors
   - Report which sensors failed
   - Partial success is acceptable

4. **Cleanup on Exit**
   - Trap SIGINT (Ctrl+C)
   - Destroy all spawned actors
   - Confirm cleanup completion

### Output Format

The script will print a summary after successful spawning:

```
Successfully spawned vehicle and sensors:
  Vehicle: ego_vehicle
  Sensors:
    - LiDAR: ego_vehicle_lidar
    - Camera: ego_vehicle_camera
    - IMU: ego_vehicle_imu
    - GNSS: ego_vehicle_gnss

ROS topics will be published as:
  /carla/ego_vehicle/vehicle_status
  /carla/ego_vehicle_lidar/point_cloud
  /carla/ego_vehicle_camera/image
  /carla/ego_vehicle_imu/imu
  /carla/ego_vehicle_gnss/gnss

Press Ctrl+C to cleanup and exit...
```

### Integration with Sensor Kit

After running the spawner script, users can launch the sensor kit with:

```bash
ros2 launch carla_sensor_kit_launch sensing.launch.xml \
  vehicle_name:=ego_vehicle \
  lidar_name:=ego_vehicle_lidar \
  camera_name:=ego_vehicle_camera \
  imu_name:=ego_vehicle_imu \
  gnss_name:=ego_vehicle_gnss
```

## Dependencies

**Python packages:**
- `carla` (CARLA Python API)
- Python 3.8+

**System requirements:**
- CARLA 0.10.0 simulator running
- ROS bridge enabled in CARLA

## Project Structure

The spawner script will be organized as a Python package managed by Rye:

```
carla_sensor_kit_launch/
├── doc/                           # Documentation
│   ├── ARCH.md
│   ├── DESIGN.md
│   ├── DESIGN_SPAWN.md           # This document
│   └── CARLA_ROS_TOPICS.md
├── carla_sensor_kit_launch/       # ROS package
├── carla_sensor_kit_description/  # ROS package
├── carla_spawner/                 # Python package for spawner script
│   ├── .python-version           # Python version (3.8+)
│   ├── .gitignore               # Python-specific ignores
│   ├── pyproject.toml           # Rye project configuration
│   ├── requirements.lock        # Locked dependencies
│   ├── requirements-dev.lock    # Dev dependencies
│   ├── README.md                # Spawner usage instructions
│   ├── src/
│   │   └── carla_spawner/
│   │       ├── __init__.py
│   │       └── spawn_vehicle.py # Main spawner script
│   └── tests/                   # Unit tests (future)
│       └── __init__.py
└── CLAUDE.md

```

### File Descriptions

**carla_spawner/pyproject.toml**
```toml
[project]
name = "carla-spawner"
version = "0.1.0"
description = "Vehicle and sensor spawning tool for CARLA-Autoware integration"
dependencies = [
    "carla @ file:///${PROJECT_ROOT}/carla-simulator/PythonAPI/carla/dist/carla-0.10.0-py3-none-any.whl",
]
requires-python = ">=3.8"
readme = "README.md"
authors = [
    { name = "Your Name", email = "your.email@example.com" }
]

[tool.rye]
managed = true
dev-dependencies = [
    "pytest>=7.0",
    "black>=22.0",
    "ruff>=0.1.0",
]

[project.scripts]
carla-spawn = "carla_spawner.spawn_vehicle:main"

[build-system]
requires = ["hatchling"]
build-backend = "hatchling.build"
```

**Note**: The `${PROJECT_ROOT}` should be replaced with the actual absolute path to the repository root when creating the file. Alternatively, use a relative path:
```toml
dependencies = [
    "carla @ file:///../carla-simulator/PythonAPI/carla/dist/carla-0.10.0-py3-none-any.whl",
]
```

**carla_spawner/README.md**
- Installation instructions
- Usage examples
- Troubleshooting guide
- Integration with sensor kit

**src/carla_spawner/__init__.py**
- Package initialization
- Version information

**src/carla_spawner/spawn_vehicle.py**
- Main script implementation
- All functionality in single file for simplicity

### Development Workflow

1. **Setup Python environment**
   ```bash
   cd carla_spawner
   rye sync
   ```

2. **Run the spawner**
   ```bash
   # Using Rye
   rye run python src/carla_spawner/spawn_vehicle.py --name ego_vehicle
   
   # Or after installation
   rye run carla-spawn --name ego_vehicle
   ```

3. **Development commands**
   ```bash
   # Format code
   rye run black src/
   
   # Lint code
   rye run ruff src/
   
   # Run tests (future)
   rye run pytest
   ```

## Future Enhancements

1. **Configuration File Support**
   - YAML file for sensor configurations
   - Multiple vehicle profiles

2. **Sensor Variants**
   - Support different LiDAR models
   - Multiple camera positions
   - Radar sensor option

3. **Launch File Generation**
   - Auto-generate launch command with correct parameters
   - Export parameter file for sensor kit

4. **Validation**
   - Verify ROS topics are being published
   - Check sensor data rates

5. **Multi-Vehicle Support**
   - Spawn multiple vehicles with unique names
   - Coordinate sensor naming across vehicles