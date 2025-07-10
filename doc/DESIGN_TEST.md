# Testing Design for CARLA Sensor Kit Launch

## Overview

This document outlines the testing strategy for the CARLA sensor kit launch package. The tests ensure proper integration between CARLA simulator sensors and ROS2/Autoware, validating topic remapping, sensor poses, and core functionality.

## Test Categories

### 1. Topic Remapping Tests

#### Purpose
Verify that sensor topics from CARLA are correctly remapped to expected Autoware topics.

#### Test Cases

**1.1 LiDAR Topic Remapping**
- **Test**: Verify `/carla/ego_vehicle/lidar` → `/sensing/lidar/top/pointcloud_raw`
- **Method**: Launch test that publishes to CARLA topic and subscribes to remapped topic
- **Validation**: Message count and content match between source and destination

**1.2 Camera Topic Remapping**
- **Test**: Verify camera topics are remapped correctly for each camera
  - Front: `/carla/ego_vehicle/camera/front` → `/sensing/camera/front/image_raw`
  - Rear: `/carla/ego_vehicle/camera/rear` → `/sensing/camera/rear/image_raw`
  - Left/Right: Similar pattern
- **Method**: Topic echo comparison test
- **Validation**: Image dimensions and encoding preserved

**1.3 IMU Topic Remapping**
- **Test**: Verify `/carla/ego_vehicle/imu` → `/sensing/imu/tamagawa/imu_raw`
- **Method**: Message type and frequency validation
- **Validation**: IMU data fields (orientation, angular velocity, linear acceleration) preserved

**1.4 GNSS Topic Remapping**
- **Test**: Verify `/carla/ego_vehicle/gnss` → `/sensing/gnss/ublox/nav_sat_fix`
- **Method**: GPS coordinate validation
- **Validation**: Lat/lon/alt values passed through correctly

### 2. Sensor Pose Validation Tests

#### Purpose
Ensure CARLA sensor poses match the transforms defined in ROS sensor descriptions.

#### Test Cases

**2.1 Static Transform Validation**
- **Test**: Verify sensor transforms match URDF/xacro definitions
- **Method**: 
  ```python
  # Compare tf2 static transforms with expected values from sensor_kit descriptions
  # Check translation (x, y, z) and rotation (roll, pitch, yaw)
  ```
- **Validation**: Transform differences within tolerance (< 1mm translation, < 0.1° rotation)

**2.2 TF Tree Consistency**
- **Test**: Validate complete TF tree from base_link to each sensor frame
- **Method**: tf2 buffer lookup and chain validation
- **Validation**: No missing transforms, proper parent-child relationships

**2.3 Dynamic Transform Updates**
- **Test**: Verify sensor transforms update correctly with vehicle movement
- **Method**: Monitor transform timestamps and continuity
- **Validation**: Transform updates at expected frequency (>10Hz)

### 3. Unit Tests

#### 3.1 Launch File Tests
- **Test**: Validate launch file parameter parsing and node spawning
- **Method**: `launch_testing` framework
- **Coverage**:
  - Parameter validation (required vs optional)
  - Node lifecycle (spawn, configure, activate)
  - Conditional launching based on parameters

#### 3.2 Configuration Loading Tests
- **Test**: Verify YAML configuration files load correctly
- **Method**: Parameter server validation
- **Coverage**:
  - Sensor calibration parameters
  - Processing pipeline configurations
  - Driver-specific settings

#### 3.3 Pointcloud Preprocessing Tests
- **Test**: Validate pointcloud preprocessing pipeline
- **Method**: Mock pointcloud data through pipeline
- **Coverage**:
  - Cropping boundaries
  - Downsampling rates
  - Outlier removal thresholds

### 4. Integration Tests

#### 4.1 End-to-End Sensor Pipeline
- **Test**: Full sensor data flow from CARLA to Autoware
- **Method**: Launch complete system and verify data flow
- **Validation**: 
  - All topics publishing at expected rates
  - Data format compatibility with Autoware nodes
  - No dropped messages under normal load

#### 4.2 Multi-Sensor Synchronization
- **Test**: Verify time synchronization between sensors
- **Method**: Compare timestamps across sensor messages
- **Validation**: Time difference < 50ms between synchronized sensors

#### 4.3 Resource Usage Tests
- **Test**: Monitor CPU/memory usage under various sensor configurations
- **Method**: System monitoring during operation
- **Validation**: Resource usage within acceptable limits

### 5. Performance Tests

#### 5.1 Latency Measurements
- **Test**: Measure end-to-end latency from CARLA to processed output
- **Method**: Timestamp injection and tracking
- **Metrics**:
  - LiDAR: < 100ms from CARLA to preprocessed pointcloud
  - Camera: < 50ms from CARLA to rectified image
  - IMU/GNSS: < 20ms from CARLA to filtered output

#### 5.2 Throughput Tests
- **Test**: Validate system handles expected data rates
- **Method**: Stress testing with maximum sensor configurations
- **Validation**:
  - LiDAR: 10Hz sustained
  - Cameras: 30Hz per camera
  - IMU: 100Hz
  - GNSS: 10Hz

### 6. Failure Mode Tests

#### 6.1 Sensor Dropout Handling
- **Test**: System behavior when sensors stop publishing
- **Method**: Selective sensor shutdown during operation
- **Validation**: Graceful degradation, appropriate error messages

#### 6.2 Invalid Data Handling
- **Test**: Response to malformed or out-of-range sensor data
- **Method**: Inject invalid data patterns
- **Validation**: Data validation and filtering works correctly

## Test Implementation

### Directory Structure
```
carla_sensor_kit_launch/
├── test/
│   ├── unit/
│   │   ├── test_launch_files.py
│   │   ├── test_config_loading.py
│   │   └── test_pointcloud_preprocessing.py
│   ├── integration/
│   │   ├── test_topic_remapping.py
│   │   ├── test_sensor_poses.py
│   │   └── test_sensor_pipeline.py
│   └── performance/
│       ├── test_latency.py
│       └── test_throughput.py
```

### Test Execution
```bash
# Run all tests
colcon test --packages-select carla_sensor_kit_launch

# Run specific test category
colcon test --packages-select carla_sensor_kit_launch --pytest-args -k "topic_remapping"

# Run with coverage
colcon test --packages-select carla_sensor_kit_launch --pytest-args --cov=carla_sensor_kit_launch
```

### Continuous Integration
- Run unit tests on every commit
- Run integration tests on pull requests
- Run performance tests nightly
- Generate test coverage reports

## Test Data

### Mock Data Generation
- Create minimal CARLA sensor message publishers for testing
- Generate representative sensor data patterns
- Include edge cases and error conditions

### Test Fixtures
- Pre-recorded sensor data bags for regression testing
- Known-good transform configurations
- Reference calibration files

## Success Criteria

1. **Coverage**: >80% code coverage for launch files and configurations
2. **Reliability**: All tests pass consistently across different environments
3. **Performance**: Latency and throughput meet specified targets
4. **Integration**: Full sensor pipeline operates without data loss
5. **Robustness**: System handles failure modes gracefully