# CARLA ROS Topics Study

## Overview

This document provides a comprehensive analysis of CARLA 0.10.0's native ROS 2 integration, including supported topics, message types, configuration requirements, and capabilities assessment.

## Study Methodology

### Analysis Approach

The study was conducted through systematic testing using custom Python scripts and ROS 2 command-line tools:

1. **Topic Discovery**: Automated listing and analysis of available ROS topics
2. **Message Examination**: Detailed inspection of topic message types and content
3. **Sensor Integration**: Testing various sensor configurations for ROS enablement
4. **Configuration Validation**: Verification of required attributes and methods
5. **Capability Assessment**: Evaluation of simulation control and state management features

### Test Environment

- **CARLA Version**: 0.10.0-Linux-Shipping
- **ROS Distribution**: Humble
- **Test Date**: 2025-06-27
- **Server Configuration**: Headless mode with `--ros2` flag
- **Port**: 3000 (default CARLA RPC port)

### Analysis Scripts

Six Python scripts were developed for comprehensive topic analysis:

| Script                       | Purpose                                                         |
|------------------------------|-----------------------------------------------------------------|
| `list_topics.py`             | Lists all available ROS 2 topics                                |
| `examine_topics.py`          | Examines topic details and message types                        |
| `sample_topics.py`           | Subscribes to topics and samples message data                   |
| `spawn_vehicle.py`           | Spawns vehicles and sensors to trigger topic publication        |
| `correct_ros_integration.py` | Implements proper ROS 2 integration following official examples |
| `test_ros_topics.py`         | Monitors ROS topics during simulation with proper setup         |

## Key Findings

### Critical Discovery: Sensor Configuration Requirements

**Most Important Finding**: CARLA sensors do NOT automatically publish ROS topics. Explicit configuration is required:

1. **Set ROS Attributes**:
   ```python
   # For vehicle
   vehicle_bp.set_attribute('role_name', 'hero')
   vehicle_bp.set_attribute('ros_name', 'hero')
   
   # For sensor  
   sensor_bp.set_attribute('role_name', 'camera')
   sensor_bp.set_attribute('ros_name', 'camera')
   ```

2. **Enable ROS Publishing**:
   ```python
   sensor.enable_for_ros()  # Essential method call
   ```

Without these steps, no CARLA-specific topics will appear in the ROS topic list.

### Available Standard ROS Topics

When CARLA server runs with `--ros2` flag, these standard topics are always available:

| Topic               | Message Type                        | Description                     |
|---------------------|-------------------------------------|---------------------------------|
| `/clock`            | `rosgraph_msgs/Clock`               | Simulation time synchronization |
| `/parameter_events` | `rcl_interfaces/msg/ParameterEvent` | ROS parameter change events     |
| `/rosout`           | `rcl_interfaces/msg/Log`            | ROS logging messages            |

### CARLA-Specific Topics (When Properly Configured)

#### Camera Topics

**RGB Camera**:
- **Topic**: `/carla/{vehicle_ros_name}/{sensor_ros_name}/image`
- **Message Type**: `sensor_msgs/Image`
- **Camera Info**: `/carla/{vehicle_ros_name}/{sensor_ros_name}/camera_info`
- **Message Type**: `sensor_msgs/CameraInfo`
- **Sensor Type**: `sensor.camera.rgb`

**Depth Camera**:
- **Topic**: `/carla/{vehicle_ros_name}/{sensor_ros_name}/image`
- **Message Type**: `sensor_msgs/Image`
- **Sensor Type**: `sensor.camera.depth`

**Semantic Segmentation**:
- **Topic**: `/carla/{vehicle_ros_name}/{sensor_ros_name}/image`
- **Message Type**: `sensor_msgs/Image`
- **Sensor Type**: `sensor.camera.semantic_segmentation`

#### LiDAR Topics

**Standard LiDAR**:
- **Topic**: `/carla/{vehicle_ros_name}/{sensor_ros_name}`
- **Message Type**: `sensor_msgs/PointCloud2`
- **Sensor Type**: `sensor.lidar.ray_cast`

**Semantic LiDAR**:
- **Topic**: `/carla/{vehicle_ros_name}/{sensor_ros_name}`
- **Message Type**: `sensor_msgs/PointCloud2`
- **Sensor Type**: `sensor.lidar.ray_cast_semantic`

#### Navigation Topics

**GNSS/GPS**:
- **Topic**: `/carla/{vehicle_ros_name}/{sensor_ros_name}`
- **Message Type**: `sensor_msgs/NavSatFix`
- **Sensor Type**: `sensor.other.gnss`

**IMU**:
- **Topic**: `/carla/{vehicle_ros_name}/{sensor_ros_name}`
- **Message Type**: `sensor_msgs/Imu`
- **Sensor Type**: `sensor.other.imu`

#### Transform Topics

**Coordinate Frames**:
- **Topic**: `/tf`
- **Message Type**: `tf2_msgs/TFMessage`
- **Description**: Coordinate frame transformations between vehicle and sensor frames

### Topic Naming Convention

CARLA uses this naming pattern for sensor topics:
```
/carla/{vehicle_ros_name}/{sensor_ros_name}[/subtype]
```

Examples:
- `/carla/hero/rgb/image` - RGB camera image
- `/carla/hero/rgb/camera_info` - Camera calibration info
- `/carla/hero/lidar` - LiDAR point cloud
- `/carla/hero/gnss` - GPS coordinates

## Configuration Requirements

### Essential Sensor Setup

For any sensor to publish ROS topics, this exact sequence is required:

```python
import carla

# Connect to CARLA
client = carla.Client('localhost', 3000)
world = client.get_world()

# Spawn vehicle with ROS configuration
vehicle_bp = world.get_blueprint_library().find('vehicle.tesla.model3')
vehicle_bp.set_attribute('role_name', 'hero')
vehicle_bp.set_attribute('ros_name', 'hero')
vehicle = world.spawn_actor(vehicle_bp, spawn_point)

# Configure sensor with ROS attributes
camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
camera_bp.set_attribute('role_name', 'rgb')
camera_bp.set_attribute('ros_name', 'rgb')

# Spawn and enable sensor for ROS
camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
camera.enable_for_ros()  # Critical step - topics won't appear without this
```

### JSON Configuration Example

CARLA also supports JSON-based sensor configuration:

```json
{
    "type": "vehicle.lincoln.mkz", 
    "id": "hero",
    "sensors": [
        {
            "type": "sensor.camera.rgb",
            "id": "rgb",
            "spawn_point": {"x": -4.5, "y": 0.0, "z": 2.5, "roll": 0.0, "pitch": 20.0, "yaw": 0.0},
            "attributes": {
                "image_size_x": 400,
                "image_size_y": 200,
                "fov": 90.0
            }
        },
        {
            "type": "sensor.lidar.ray_cast",
            "id": "lidar", 
            "spawn_point": {"x": 0.0, "y": 0.0, "z": 2.4, "roll": 0.0, "pitch": 0.0, "yaw": 0.0},
            "attributes": {
                "range": 85,
                "channels": 64,
                "points_per_second": 600000,
                "rotation_frequency": 20,
                "upper_fov": 10,
                "lower_fov": -30
            }
        }
    ]
}
```

## Supported Sensor Types

### Camera Sensors
- `sensor.camera.rgb` - RGB camera
- `sensor.camera.depth` - Depth camera
- `sensor.camera.semantic_segmentation` - Semantic segmentation
- `sensor.camera.instance_segmentation` - Instance segmentation
- `sensor.camera.optical_flow` - Optical flow
- `sensor.camera.normals` - Surface normals

### LiDAR Sensors
- `sensor.lidar.ray_cast` - Standard LiDAR
- `sensor.lidar.ray_cast_semantic` - Semantic LiDAR

### Other Sensors
- `sensor.other.gnss` - GPS/GNSS
- `sensor.other.imu` - Inertial Measurement Unit
- `sensor.other.collision` - Collision detection
- `sensor.other.lane_invasion` - Lane invasion detection
- `sensor.other.radar` - Radar sensor

## ROS API Methods

### Sensor Control
```python
# Enable ROS publishing
sensor.enable_for_ros()

# Disable ROS publishing  
sensor.disable_for_ros()

# Check if ROS publishing is enabled
is_enabled = sensor.is_enabled_for_ros()
```

## Simulation Control and Server State Analysis

### Current Native ROS 2 Capabilities

**What CARLA 0.10.0 Native ROS 2 CAN do**:
- ✅ Publish sensor data (cameras, LiDAR, IMU, GNSS)
- ✅ Provide simulation time via `/clock` topic
- ✅ Accept vehicle control commands (when implemented)
- ✅ Broadcast coordinate transforms via `/tf`
- ✅ Support all major RMW implementations

**What it CANNOT do natively**:
- ❌ Control simulation state (play/pause/step)
- ❌ Provide world state information via topics
- ❌ Control weather via ROS topics/services
- ❌ Manage traffic lights via ROS
- ❌ Provide map information via ROS
- ❌ Reset or restart simulation via ROS
- ❌ Offer ROS services for world management

### Available Server State Topics

**Current Status**: Limited server state information available through native integration.

| Topic | Message Type | Availability | Description |
|-------|--------------|--------------|-------------|
| `/clock` | `rosgraph_msgs/Clock` | ✅ Always | Simulation time |
| `/parameter_events` | `rcl_interfaces/msg/ParameterEvent` | ✅ Always | ROS parameter changes |
| `/rosout` | `rcl_interfaces/msg/Log` | ✅ Always | ROS logging |
| `/carla/status` | N/A | ❌ Not available | Server status |
| `/carla/world_info` | N/A | ❌ Not available | World information |
| `/carla/control` | N/A | ❌ Not available | Simulation control |

### Simulation Control Limitations

**Missing Control Capabilities**:
1. **No play/pause control** - Cannot control simulation state via ROS
2. **No step simulation** - Cannot advance simulation by single steps
3. **No world reset** - Cannot reset simulation via ROS commands
4. **No weather control** - Cannot change weather conditions via ROS
5. **No traffic management** - Cannot control traffic lights or traffic flow
6. **No map switching** - Cannot change maps via ROS interface

### Comparison: Native vs External ROS Bridge

| Feature | Native ROS 2 (0.10.0) | External CARLA ROS Bridge |
|---------|------------------------|--------------------------|
| Sensor data publishing | ✅ Full support | ✅ Full support |
| Vehicle control | ✅ Basic support | ✅ Full support |
| Simulation control | ❌ Not available | ✅ Full control |
| World state topics | ❌ Limited | ✅ Comprehensive |
| Traffic light control | ❌ Not available | ✅ Available |
| Weather control | ❌ Not available | ✅ Available |
| Map management | ❌ Not available | ✅ Available |
| ROS services | ❌ Minimal | ✅ Extensive |

## Investigation Results

### Topic Discovery Issues Initially Encountered

**Problem**: Initial testing showed only standard ROS topics, no CARLA-specific topics.

**Investigation Process**:
1. **Basic vehicle spawning** - No topics appeared
2. **Sensor attachment** - Still no CARLA topics
3. **Official example analysis** - Discovered missing `enable_for_ros()` calls
4. **Proper configuration implementation** - Topics appeared successfully

**Root Cause**: CARLA's native ROS 2 integration requires explicit enablement:
- Both `role_name` and `ros_name` attributes must be set
- `enable_for_ros()` method must be called on each sensor
- Without these steps, sensors operate normally but don't publish ROS topics

### Testing Validation

**Successful Configuration Results**:
- **Topics Found**: 4+ CARLA-specific topics when properly configured
- **Message Reception**: Successful message sampling from sensor topics
- **Transform Broadcasting**: Proper `/tf` frame relationships
- **Timing**: Real-time sensor data at expected frequencies

## Recommendations

### For Sensor Data Collection

**Native ROS 2 integration is sufficient** when you need:
- Camera, LiDAR, IMU, GNSS sensor data
- Real-time sensor feeds
- Standard ROS message formats
- Integration with ROS 2 ecosystem

**Required setup**:
```python
# Essential configuration for each sensor
sensor_bp.set_attribute('role_name', 'sensor_name')
sensor_bp.set_attribute('ros_name', 'sensor_name')
sensor = world.spawn_actor(sensor_bp, transform, attach_to=vehicle)
sensor.enable_for_ros()  # Must call this method
```

### For Full Simulation Control

**External CARLA ROS Bridge required** when you need:
- Simulation play/pause/step control
- World state management
- Traffic light control
- Weather manipulation
- Map switching
- Comprehensive ROS services

**Installation**:
```bash
# Install CARLA ROS Bridge (separate package)
sudo apt install ros-humble-carla-ros-bridge
# or build from source: carla-simulator/ros-bridge
```

## Message Type Dependencies

Required ROS packages for CARLA topics:

```bash
# Essential message packages
sudo apt install ros-humble-sensor-msgs      # Camera, LiDAR, IMU, GNSS
sudo apt install ros-humble-geometry-msgs    # Vehicle control
sudo apt install ros-humble-tf2-msgs        # Coordinate transforms
sudo apt install ros-humble-rosgraph-msgs   # Clock synchronization

# Optional packages
sudo apt install ros-humble-nav-msgs        # Navigation
sudo apt install ros-humble-ackermann-msgs  # Ackermann steering
```

## RMW Compatibility

**Test Results**: All major RMW implementations are compatible with CARLA's native ROS 2 integration:

- ✅ **rmw_fastrtps_cpp** - Default, fully tested
- ✅ **rmw_cyclonedx_cpp** - Fully compatible  
- ✅ **rmw_connextdds** - Fully compatible
- ✅ **rmw_zenoh_cpp** - Fully compatible

**Performance**: No significant performance differences observed between RMW implementations for CARLA use cases.

## Visualization with RViz

CARLA provides RViz integration capabilities:

```bash
# Launch RViz with CARLA-compatible configuration
rviz2 -d carla_config.rviz
```

**Supported visualizations**:
- RGB camera image displays
- LiDAR point cloud rendering
- Coordinate frame visualization
- Vehicle pose and trajectory
- Sensor mounting relationships

## Troubleshooting Guide

### Common Issues and Solutions

**No CARLA topics appearing**:
```python
# Solution: Ensure proper sensor configuration
sensor_bp.set_attribute('role_name', 'camera')
sensor_bp.set_attribute('ros_name', 'camera')  
sensor.enable_for_ros()  # Essential call
```

**Topics appear but no messages**:
- Check sensor is properly attached to vehicle
- Verify vehicle is spawned in valid location
- Ensure simulation is running (not paused)

**RMW compatibility issues**:
```bash
# Ensure same RMW for server and client
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

**Port conflicts**:
```bash
# Check CARLA port availability
sudo netstat -tlnp | grep :3000
sudo fuser -k 3000/tcp  # Kill processes using port
```

## Future Considerations

### Potential Enhancements

1. **Extended Native Support**: Future CARLA versions may add simulation control topics
2. **Service Integration**: ROS services for world management could be added
3. **Dynamic Configuration**: Runtime sensor configuration via ROS parameters
4. **Performance Optimization**: Improved topic publication efficiency

### Compatibility Expectations

- **ROS 2 Support**: Likely to continue in future CARLA versions
- **RMW Compatibility**: Should remain broad across implementations
- **API Stability**: `enable_for_ros()` method expected to remain in future versions

## Conclusion

CARLA 0.10.0's native ROS 2 integration provides robust sensor data publishing capabilities with broad RMW compatibility. The key requirement is proper sensor configuration with explicit ROS enablement. While simulation control capabilities are limited, the native integration is sufficient for most autonomous vehicle research applications focused on sensor data processing and perception algorithms.

For comprehensive simulation management, the external CARLA ROS Bridge package remains necessary, but for sensor-focused applications, the native integration offers excellent performance and simplicity.

---

*Study completed on 2025-06-27 using CARLA 0.10.0-Linux-Shipping with ROS 2 Humble*
