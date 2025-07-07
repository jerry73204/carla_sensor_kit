# CARLA Sensor Kit Architecture

## System Architecture

```
┌─────────────────────────┐
│   CARLA Simulator       │
│      (0.10.0)           │
│ ┌─────────────────────┐ │
│ │  Vehicle Actor      │ │
│ │ ┌─────┐ ┌─────┐    │ │
│ │ │LiDAR│ │ CAM │    │ │
│ │ │Actor│ │Actor│    │ │
│ │ └──┬──┘ └──┬──┘    │ │
│ │ ┌──┴──┐ ┌──┴──┐    │ │
│ │ │ IMU │ │GNSS │    │ │
│ │ │Actor│ │Actor│    │ │
│ │ └──┬──┘ └──┬──┘    │ │
│ └────┼───────┼────────┘ │
└──────┼───────┼──────────┘
       │       │
       ▼       ▼
┌─────────────────────────┐
│  CARLA ROS Bridge       │
│  Publishing Topics:     │
│  /carla/<actor>/<type>  │
└───────────┬─────────────┘
            │
            ▼
┌─────────────────────────┐
│  CARLA Sensor Kit       │
│  Launch Package         │
│ ┌─────────────────────┐ │
│ │  Topic Remapping    │ │
│ │  & Preprocessing    │ │
│ └─────────────────────┘ │
└───────────┬─────────────┘
            │
            ▼
┌─────────────────────────┐
│     Autoware            │
│  /sensing/<type>        │
└─────────────────────────┘
```

## Package Dependencies

```
carla_sensor_kit_launch
├── carla_sensor_kit_description
├── autoware_gnss_poser
├── autoware_pointcloud_preprocessor
├── autoware_vehicle_velocity_converter
├── topic_tools
├── ublox_gps
└── usb_cam

carla_sensor_kit_description
└── velodyne_description
```

## Data Flow Architecture

### LiDAR Pipeline
```
CARLA LiDAR Actor
    │
    ├─→ /carla/<lidar_name>/point_cloud
    │
    ├─→ [Topic Remapping]
    │
    ├─→ /sensing/lidar/top/pointcloud_raw
    │
    ├─→ [Pointcloud Preprocessor]
    │   ├── Ring Outlier Filter
    │   ├── Distortion Corrector
    │   └── Mirror Filter
    │
    └─→ /sensing/lidar/top/pointcloud
```

### Camera Pipeline
```
CARLA Camera Actor
    │
    ├─→ /carla/<camera_name>/image
    │
    ├─→ [Topic Remapping]
    │
    └─→ /sensing/camera/<position>/image_raw
```

### IMU Pipeline
```
CARLA IMU Actor
    │
    ├─→ /carla/<imu_name>/imu
    │
    ├─→ [Topic Remapping]
    │
    ├─→ /sensing/imu/tamagawa/imu_raw
    │
    ├─→ [IMU Corrector]
    │
    └─→ /sensing/imu/tamagawa/imu_data
```

### GNSS Pipeline
```
CARLA GNSS Actor
    │
    ├─→ /carla/<gnss_name>/gnss
    │
    ├─→ [Topic Remapping]
    │
    ├─→ /sensing/gnss/ublox/nav_sat_fix
    │
    └─→ [GNSS Poser]
        │
        └─→ /sensing/gnss/pose
```

## Launch Hierarchy

```
sensing.launch.xml
├── lidar.launch.xml
│   ├── pointcloud_preprocessor.launch.py
│   └── [Container with preprocessing nodes]
├── camera.launch.xml (commented out by default)
│   └── [USB camera nodes]
├── imu.launch.xml
│   └── [IMU corrector node]
├── gnss.launch.xml
│   └── [GNSS driver and poser]
└── vehicle_velocity_converter.launch.xml
```

## Configuration Structure

```
carla_sensor_kit_launch/
├── config/
│   ├── concatenate_and_time_sync_node.param.yaml
│   ├── diagnostic_aggregator/
│   │   └── sensor_kit.param.yaml
│   └── dummy_diag_publisher/
│       └── sensor_kit.param.yaml
└── data/
    └── traffic_light_camera.yaml

carla_sensor_kit_description/
└── config/
    ├── sensor_kit_calibration.yaml
    ├── sensors_calibration.yaml
    └── imu_corrector.param.yaml
```

## Key Design Decisions

### 1. Actor Name Parameterization
- Actor names must be provided at launch time
- Enables flexibility for different CARLA setups
- Avoids hardcoding simulator-specific names

### 2. Topic Namespace Strategy
- Input: `/carla/<actor_name>/<sensor_type>`
- Output: `/sensing/<sensor_type>/<sensor_name>/<data_type>`
- Maintains clear separation between simulator and Autoware domains

### 3. Container-based Processing
- LiDAR preprocessing runs in a composable node container
- Enables efficient intra-process communication
- Reduces latency for pointcloud operations

### 4. Modular Sensor Activation
- Each sensor type can be enabled/disabled via launch arguments
- Camera driver is commented out by default
- Supports partial sensor configurations

### 5. Calibration Separation
- Static transforms in URDF/xacro files
- Dynamic calibration in YAML configurations
- Enables runtime calibration updates without rebuilding