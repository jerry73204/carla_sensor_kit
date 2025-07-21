# CARLA Sensor Kit Design Document

## Overview

This document describes the design of the CARLA sensor kit launch package, which provides integration between CARLA 0.10.0 simulator and Autoware. The sensor kit acts as a bridge layer that remaps and relays sensor topics from CARLA's actor-based simulation to Autoware's expected sensor interfaces.

## Purpose

The CARLA sensor kit serves the following purposes:

1. **Topic Translation**: Remaps CARLA's actor-based topic naming scheme to Autoware's standardized sensor topic names
2. **Sensor Configuration**: Provides launch configurations for various sensor types (LiDAR, camera, IMU, GNSS)
3. **Calibration Management**: Manages sensor calibration data and transformations between CARLA and Autoware coordinate systems
4. **Preprocessing Pipeline**: Sets up necessary preprocessing nodes for sensor data (e.g., pointcloud filtering)

## Design Principles

### 1. Modularity
- Each sensor type has its own launch file
- Configuration is separated from launch logic
- Sensor descriptions are maintained independently

### 2. Flexibility
- Actor names are parameterized and provided at launch time
- Launch arguments control which sensors are activated
- Preprocessing steps can be enabled/disabled

### 3. Compatibility
- Maintains compatibility with Autoware's sensing interface
- Preserves CARLA's simulation fidelity
- Supports standard ROS2 patterns and conventions

## Workflow

### 1. Simulation Setup
```
CARLA Simulator → Create Vehicle Actor → Attach Sensor Actors
                                              ↓
                                    sensor_config_loader.py
                                    (loads transforms from YAML)
```

### 2. Launch Sensor Kit
```
User provides actor names → Launch files remap topics → Autoware receives data
```

### 3. Data Flow
```
CARLA Actor Topics → Topic Remapping → Preprocessing → Autoware Topics
```

### 4. Unified Configuration Flow
```
YAML Config Files → sensor_config_loader.py → CARLA Spawner
        ↓                                           ↓
    URDF/Xacro  ←────── Same Source ──────→  Sensor Actors
```

## Key Components

### Launch Files
- **sensing.launch.xml**: Main orchestrator that includes all sensor launches
- **lidar.launch.xml**: Handles LiDAR topic remapping and preprocessing
- **camera.launch.xml**: Manages camera topic remapping
- **imu.launch.xml**: Remaps IMU data topics
- **gnss.launch.xml**: Handles GNSS/GPS topic translation
- **pointcloud_preprocessor.launch.py**: Python launch for composable pointcloud processing

### Configuration Files
- **sensor_kit_calibration.yaml**: Individual sensor mounting positions relative to sensor_kit_base_link
- **sensors_calibration.yaml**: Sensor kit base position relative to vehicle base_link
- **imu_corrector.param.yaml**: IMU correction parameters
- **concatenate_and_time_sync_node.param.yaml**: Multi-LiDAR synchronization settings
- **diagnostic aggregator configs**: Health monitoring configurations

### URDF/Xacro Files
- **sensor_kit.xacro**: Defines the complete sensor kit assembly
- **sensors.xacro**: Individual sensor descriptions and transforms

### Integration Scripts
- **sensor_config_loader.py**: Loads sensor transforms from YAML for CARLA spawner
- **carla.rs**: CARLA server management (start/stop/status)
- **ros_env.sh**: ROS2 environment setup helper

## Interface Contract

### Input (from CARLA)
- Actor-based topic names: `/carla/<actor_name>/<sensor_type>`
- CARLA coordinate system and units
- Raw sensor data as published by CARLA ROS bridge

### Output (to Autoware)
- Standardized topic names: `/sensing/<sensor_type>/<data_type>`
- Autoware coordinate system (ROS REP-103)
- Preprocessed and calibrated sensor data

## Extension Points

1. **New Sensor Types**: Add new launch files following the existing pattern
2. **Custom Preprocessing**: Insert additional nodes in the launch pipeline
3. **Alternative Simulators**: Replace CARLA-specific remapping with other simulator interfaces
4. **Calibration Updates**: Modify YAML files without changing launch logic
5. **Coordinate System Conversion**: Extend sensor_config_loader.py for different coordinate systems
6. **Multi-vehicle Support**: Parameterize sensor kit for multiple vehicle instances

## Implementation Status

### Completed
- Launch file structure for all sensor types
- YAML configuration files with sensor calibrations
- URDF/xacro descriptions for sensor mounting
- Topic remapping from CARLA to Autoware namespaces
- Unified sensor configuration loader (sensor_config_loader.py)
- CARLA server management scripts

### In Progress
- Integration with CARLA spawner using unified configurations
- Comprehensive workflow automation script

### Future Work
- Camera driver integration (currently commented out)
- Additional sensor types (radar, ultrasonic)
- Performance optimization for high-frequency sensors
- Cloud deployment support

## Dynamic Configuration System

### Overview

The dynamic configuration system generates ROS2 parameters and descriptions from CARLA simulation data at runtime without creating files on disk. This approach ensures configurations always match the actual simulated vehicle and sensor parameters.

### Design Goals

1. **Zero File Generation**: All configurations exist only in memory
2. **Runtime Flexibility**: Parameters reflect actual CARLA state
3. **Type Safety**: Strong typing for all parameter mappings
4. **Backward Compatibility**: Maintains Autoware interface expectations

### Parameter Extraction Architecture

```
CARLA Simulator
    ↓
Parameter Extraction Layer
    ├── Vehicle Parameter Extractor
    └── Sensor Parameter Extractor
         ↓
    Data Transformer
    ├── Coordinate System Converter
    ├── Unit Converter
    └── Parameter Calculator
         ↓
ROS Description Generator
    ├── YAML Parameter Dictionaries
    └── URDF String Generation
         ↓
    Autoware-Compatible
    Runtime Configuration
```

### Key Components

#### 1. Vehicle Parameter Extractor

Extracts from CARLA:
- Vehicle bounding box dimensions
- Physics control parameters (mass, drag, wheels)
- Wheel configurations and positions

Calculates derived parameters:
- `wheel_base`: Distance between front and rear axles
- `wheel_tread`: Distance between left and right wheels
- Overhangs: Distances from wheels to vehicle edges

#### 2. Sensor Parameter Extractor

Extracts from CARLA sensor actors:
- Camera: resolution, FOV, frame rate
- LiDAR: channels, range, rotation frequency
- IMU/GNSS: Update rates and noise parameters
- Transform: Position and orientation relative to vehicle

#### 3. Coordinate Transformer

Handles conversions between:
- CARLA (Unreal Engine): X-forward, Y-right, Z-up, left-handed
- ROS (REP-103): X-forward, Y-left, Z-up, right-handed
- Units: Centimeters to meters

#### 4. Dynamic Configuration Generator

Generates at runtime:
- Vehicle parameter dictionaries
- Sensor configuration mappings
- Complete URDF as string (no file I/O)
- Transform trees for tf2

### Launch Integration

```python
def generate_dynamic_launch_setup(context):
    # Connect to CARLA and find vehicle
    vehicle = find_vehicle_by_role_name()

    # Generate configurations
    config_gen = DynamicConfigGenerator(world, vehicle)
    vehicle_params = config_gen.generate_vehicle_parameters()
    robot_description = config_gen.generate_urdf_string()

    # Launch nodes with dynamic parameters
    return [
        Node(
            package='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),
        Node(
            package='carla_vehicle_interface',
            parameters=[vehicle_params]
        )
    ]
```

### Parameter Mapping Strategy

| CARLA Parameter | ROS Parameter | Transformation |
|----------------|---------------|----------------|
| bounding_box.extent | vehicle_height/width/length | * 2.0 (half-extents), cm to m |
| wheels[i].radius | wheel_radius | cm to m |
| wheels positions | wheel_base, wheel_tread | Calculate distances |
| physics.mass | mass | Direct (kg) |
| physics.max_steer_angle | max_steer_angle | deg to rad |

### Benefits

1. **Accuracy**: Parameters match exactly what's simulated
2. **Flexibility**: Support any CARLA vehicle blueprint
3. **Consistency**: Single source of truth (CARLA)
4. **Maintainability**: No generated files to manage
5. **Container-friendly**: Works in read-only filesystems

### Implementation Guidelines

1. **Error Handling**: Use defaults when CARLA data unavailable
2. **Caching**: Cache extracted parameters to reduce API calls
3. **Validation**: Ensure parameters within reasonable ranges
4. **Thread Safety**: Protect CARLA access in multi-vehicle scenarios
