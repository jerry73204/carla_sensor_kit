# CARLA-Autoware Workflow Automation

## Overview

This document describes the automated workflow for running the complete CARLA-Autoware integration stack. The workflow orchestrates CARLA simulator startup, vehicle/sensor spawning, and Autoware launch with proper sequencing and health checks.

## Workflow Components

### 1. CARLA Server Management
- Start/stop CARLA simulator with configurable options
- Health monitoring and readiness checks
- Support for headless and display modes
- Configurable RPC port and quality settings

### 2. Vehicle and Sensor Spawning
- Spawn vehicles at predefined or custom locations
- Attach sensors using unified YAML configurations
- Verify sensor actor creation and data publishing
- Support for multiple vehicles

### 3. Autoware Integration
- Launch sensor kit with proper topic remapping
- Start sensing pipeline (LiDAR, cameras, IMU, GNSS)
- Optional launch of perception, planning, and control stacks
- Monitor node health and topic frequencies

## Workflow Sequence

```
┌─────────────────────┐
│   Pre-flight        │
│   Checks            │
│ • CARLA installed   │
│ • ROS2 sourced      │
│ • Packages built    │
└──────────┬──────────┘
           │
┌──────────▼──────────┐
│   Start CARLA       │
│   Server            │
│ • Kill existing     │
│ • Launch new        │
│ • Wait for ready    │
└──────────┬──────────┘
           │
┌──────────▼──────────┐
│   Spawn Actors      │
│ • Load sensor cfg   │
│ • Create vehicles   │
│ • Attach sensors    │
│ • Verify spawning   │
└──────────┬──────────┘
           │
┌──────────▼──────────┐
│   Launch Autoware   │
│ • Source ROS env    │
│ • Launch sensing    │
│ • Start modules     │
│ • Monitor health    │
└──────────┬──────────┘
           │
┌──────────▼──────────┐
│   Runtime Monitor   │
│ • Display status    │
│ • Check topics      │
│ • Handle errors     │
│ • Graceful exit     │
└─────────────────────┘
```

## Configuration

### Command-line Options

```bash
run_carla_autoware.py [options]

Options:
  --carla-headless          Run CARLA without display
  --carla-port PORT         CARLA RPC port (default: 2000)
  --carla-map MAP           Map to load (default: Town03)
  --rmw IMPLEMENTATION      ROS middleware (default: rmw_fastrtps_cpp)
  --vehicles N              Number of vehicles (default: 1)
  --spawn-point X,Y,Z       Custom spawn location
  --autoware-modules LIST   Additional modules to launch
  --log-dir PATH           Log directory (default: ./logs)
  --timeout SECONDS        Startup timeout (default: 60)
  --no-cleanup             Skip cleanup on exit
```

### Configuration File (Optional)

```yaml
# carla_autoware_workflow.yaml
carla:
  headless: false
  port: 2000
  map: Town03
  quality_level: Low
  
spawner:
  config_path: null  # Use default sensor configs
  vehicles:
    - position: [92.0, -128.0, 0.3]
      rotation: [0, 0, 0]
    
autoware:
  rmw_implementation: rmw_fastrtps_cpp
  sensor_kit_launch: carla_sensor_kit_launch
  launch_modules:
    - sensing
    - perception  # optional
    - planning    # optional
    
monitoring:
  log_directory: "./logs"
  status_update_hz: 1.0
  topic_timeout: 5.0
```

## Health Monitoring

### Status Display

The workflow provides real-time status monitoring:

```
[CARLA-Autoware Status] 2024-01-10 10:45:23
═══════════════════════════════════════════
CARLA Server:    ✓ Running (PID: 12345, Port: 2000)
Spawner:         ✓ Complete (1 vehicle, 4 sensors)
Autoware:        ✓ Active (15 nodes)

Sensor Status:
  LiDAR Top:     ✓ 20.1 Hz [/sensing/lidar/top/pointcloud_raw]
  GNSS:          ✓ 10.0 Hz [/sensing/gnss/pose]
  IMU:           ✓ 100.2 Hz [/sensing/imu/imu_data]
  Camera:        ✓ 30.0 Hz [/sensing/camera/traffic_light/image_raw]

System Health:
  CPU: 45%  |  Memory: 8.2 GB  |  GPU: 62%

Press Ctrl+C to shutdown gracefully...
```

### Error Handling

The workflow includes comprehensive error handling:

1. **CARLA Connection Failures**
   - Retry with exponential backoff
   - Clear error messages with troubleshooting steps
   
2. **Spawn Failures**
   - Verify actor names and positions
   - Check for collisions or invalid locations
   
3. **Autoware Launch Issues**
   - Validate package builds
   - Check for missing dependencies
   - Monitor for node crashes

## Logging

All components log to separate files:

```
logs/
├── carla_server_20240110_104523.log
├── spawner_20240110_104530.log
├── autoware_20240110_104535.log
└── workflow_20240110_104520.log
```

Log rotation prevents disk space issues.

## Integration with CI/CD

The workflow supports automation:

```bash
# Run in CI with specific configuration
python run_carla_autoware.py \
  --carla-headless \
  --timeout 120 \
  --autoware-modules sensing,perception \
  --log-dir /tmp/test_logs
  
# Check exit code
if [ $? -eq 0 ]; then
  echo "Test passed"
else
  echo "Test failed"
  cat /tmp/test_logs/workflow_*.log
fi
```

## Troubleshooting

### Common Issues

1. **CARLA won't start**
   - Check DISPLAY variable for non-headless mode
   - Verify CARLA installation path
   - Ensure no other instances running

2. **Sensors not publishing**
   - Verify sensor names in YAML match spawner
   - Check CARLA-ROS bridge is active
   - Confirm topic remapping in launch files

3. **Autoware nodes crashing**
   - Check ROS_DOMAIN_ID consistency
   - Verify RMW implementation compatibility
   - Review node logs for specific errors

## Future Enhancements

1. **Docker Support**
   - Containerized workflow execution
   - Multi-container orchestration
   
2. **Remote Execution**
   - Distributed component deployment
   - Network latency compensation
   
3. **Scenario Integration**
   - OpenSCENARIO support
   - Automated test execution
   
4. **Performance Profiling**
   - Resource usage tracking
   - Bottleneck identification
   
5. **Web Dashboard**
   - Real-time status visualization
   - Remote control interface