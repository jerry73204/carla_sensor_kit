# CARLA Sensor Kit Launch - Development Roadmap

This document outlines the development phases for the CARLA sensor kit launch project, which integrates CARLA 0.10.0 simulator with Autoware through ROS2.

## Phase 1: Foundation (Current)
**Goal**: Establish basic vehicle spawning and sensor data flow

### Completed
- [x] Create project structure with ROS2 packages
- [x] Design documentation (ARCH.md, DESIGN.md)
- [x] Define CARLA ROS topic specifications
- [x] Design vehicle spawner script architecture
- [x] Create sensor kit launch files
  - [x] Implement topic remapping for each sensor type
  - [x] Add launch parameters for actor names
  - [x] Configure sensor transforms in YAML
- [x] Implement unified sensor configuration
  - [x] Create sensor_config_loader.py
  - [x] Eliminate pose duplication between spawner and YAML
  - [x] Add coordinate system conversion (ROS → CARLA)
- [x] Create helper scripts
  - [x] carla.rs for server management
  - [x] ros_env.sh for environment setup

### In Progress
- [ ] Test complete data flow from CARLA to Autoware
- [ ] Validate sensor data quality and timing

### Completed Recently
- [x] Integrate sensor config loader with CARLA spawner
  - [x] Design integration approach
  - [x] Create simple_spawn_integrated.py with YAML configs
  - [x] Test sensor placement accuracy
- [x] Implement workflow automation script
  - [x] Design workflow architecture (WORKFLOW.md)
  - [x] Implement run_carla_autoware.py
  - [x] Add health monitoring and status display
- [x] Implement testing framework
  - [x] Create test directory structure
  - [x] Set up pytest and launch_testing infrastructure
  - [x] Configure test coverage reporting
  - [x] Create initial test files for topic remapping and sensor poses

## Phase 2: Dynamic Configuration & Vehicle Interface
**Goal**: Implement dynamic sensor/vehicle configuration from CARLA and vehicle control interface

### Dynamic Sensor Configuration
- [ ] Implement sensor parameter extraction from CARLA
  - [ ] Create SensorParameterExtractor class
  - [ ] Extract camera specs (resolution, FOV, FPS)
  - [ ] Extract LiDAR specs (channels, range, frequency)
  - [ ] Extract sensor transforms from CARLA attachments
- [ ] Remove old sensor configuration code
  - [ ] Remove hardcoded sensor specs from vehicle_spawner.py
  - [ ] Delete redundant sensor config loading code
  - [ ] Update spawn scripts to use dynamic extraction
- [ ] Implement runtime parameter generation
  - [ ] Create sensor parameter dictionaries at runtime
  - [ ] Generate sensor launch configurations dynamically
  - [ ] Pass parameters to sensor driver nodes
- [ ] Testing tasks:
  - [ ] Unit tests for parameter extraction
  - [ ] Integration tests for dynamic configuration
  - [ ] Validation against hardcoded values

### Dynamic Vehicle Configuration
- [ ] Implement vehicle parameter extraction
  - [ ] Create VehicleParameterExtractor class
  - [ ] Extract vehicle dimensions from bounding box
  - [ ] Extract wheel parameters and physics
  - [ ] Calculate derived parameters (overhangs, wheel base)
- [ ] Implement URDF generation
  - [ ] Create DynamicURDFGenerator class
  - [ ] Generate URDF string from vehicle parameters
  - [ ] Include sensor mounting points
  - [ ] No file I/O - all in memory
- [ ] Testing tasks:
  - [ ] Unit tests for URDF generation
  - [ ] Validation of generated URDF
  - [ ] TF tree consistency tests

### Vehicle Interface Implementation
- [ ] Create CARLA vehicle interface node
  - [ ] Implement CARLAVehicleInterface ROS2 node
  - [ ] Subscribe to Autoware control commands
  - [ ] Translate commands to CARLA VehicleControl
  - [ ] Apply control via CARLA Python API
- [ ] Implement vehicle state publishing
  - [ ] Poll vehicle state from CARLA
  - [ ] Publish velocity status
  - [ ] Publish steering status
  - [ ] Publish gear and control mode
- [ ] Add error handling
  - [ ] Handle CARLA connection loss
  - [ ] Implement reconnection logic
  - [ ] Publish degraded status on errors
- [ ] Testing tasks:
  - [ ] Control command translation tests
  - [ ] State publishing accuracy tests
  - [ ] Error recovery tests
  - [ ] Multi-vehicle isolation tests

### Launch Integration
- [ ] Create dynamic launch system
  - [ ] Implement OpaqueFunction launch setup
  - [ ] Query CARLA for vehicle/sensor data
  - [ ] Generate parameters at launch time
  - [ ] Create nodes with dynamic configs
- [ ] Update existing launch files
  - [ ] Modify vehicle.launch.xml
  - [ ] Add vehicle interface to sensing.launch.xml
  - [ ] Remove static configuration dependencies
- [ ] Testing tasks:
  - [ ] Launch file integration tests
  - [ ] Parameter propagation tests
  - [ ] Multi-vehicle launch tests

### Migration & Cleanup
- [ ] Phase out old configuration system
  - [ ] Update documentation
  - [ ] Remove deprecated scripts
  - [ ] Clean up unused YAML files
- [ ] Backward compatibility
  - [ ] Add migration guide
  - [ ] Support legacy launch arguments
  - [ ] Deprecation warnings

## Phase 3: Sensor Integration
**Goal**: Full sensor suite working with accurate data transformation

### LiDAR Pipeline
- [ ] Implement pointcloud preprocessor launch
- [ ] Configure ring outlier filter
- [ ] Add distortion correction
- [ ] Validate coordinate frame transformations
- [ ] Test topic remapping for LiDAR data

### Camera Pipeline
- [ ] Enable camera launch file
- [ ] Configure image transport
- [ ] Add camera info publishing
- [ ] Test multiple camera positions
- [ ] Validate camera topic remapping

### IMU Integration
- [ ] Configure IMU corrector node
- [ ] Validate noise parameters
- [ ] Test data rates and latency
- [ ] Test IMU topic remapping and data integrity

### GNSS Integration
- [ ] Configure GNSS poser node
- [ ] Implement coordinate conversions
- [ ] Add UTM zone handling
- [ ] Test GNSS topic remapping and coordinate accuracy

### Testing Integration
- [ ] Implement topic remapping tests for all sensors
- [ ] Create sensor pose validation tests
- [ ] Add sensor data quality tests

## Phase 4: Calibration & Transforms
**Goal**: Accurate sensor positioning and calibration

- [ ] Create sensor mounting configuration UI/tool
- [ ] Implement dynamic TF tree generation
- [ ] Add sensor calibration file generator
- [ ] Create calibration validation tools
- [ ] Document calibration procedures
- [ ] Testing tasks:
  - [ ] Implement TF tree consistency tests
  - [ ] Add transform accuracy validation
  - [ ] Create calibration regression tests

## Phase 5: Multi-Vehicle Support
**Goal**: Support multiple vehicles in same simulation

- [ ] Extend spawner for multiple vehicles
- [ ] Implement namespace isolation for topics
- [ ] Add vehicle fleet configuration file
- [ ] Create traffic scenario scripts
- [ ] Test with 5+ simultaneous vehicles
- [ ] Testing tasks:
  - [ ] Multi-vehicle topic isolation tests
  - [ ] Performance tests with multiple vehicles
  - [ ] Resource usage monitoring tests

## Phase 6: Advanced Features
**Goal**: Production-ready features and optimizations

### Performance Optimization
- [ ] Implement zero-copy message passing
- [ ] Add configurable data decimation
- [ ] Optimize pointcloud processing pipeline
- [ ] Profile and reduce latency
- [ ] Testing tasks:
  - [ ] Latency measurement tests
  - [ ] Throughput performance tests
  - [ ] CPU/memory usage profiling

### Monitoring & Diagnostics
- [ ] Integrate diagnostic aggregator
- [ ] Add sensor health monitoring
- [ ] Create visualization dashboard
- [ ] Implement data recording tools
- [ ] Testing tasks:
  - [ ] Sensor dropout detection tests
  - [ ] Health monitoring validation
  - [ ] Diagnostic message tests

### Additional Sensors
- [ ] Add radar sensor support
- [ ] Implement ultrasonic sensors
- [ ] Add V2X communication simulation
- [ ] Support custom sensor types

## Phase 7: Testing & Validation
**Goal**: Comprehensive testing framework

### Core Testing Implementation
- [ ] Create unit tests for spawner script
  - [ ] Test sensor configuration loading
  - [ ] Test coordinate system conversions
  - [ ] Test vehicle spawning logic
- [ ] Add integration tests for launch files
  - [ ] Test launch file parameter parsing
  - [ ] Test node lifecycle management
  - [ ] Test conditional launching
- [ ] Implement sensor data validation
  - [ ] Create data quality metrics
  - [ ] Add range and rate validation
  - [ ] Implement data consistency checks

### Test Framework Setup
- [ ] Create test directory structure as per DESIGN_TEST.md
- [ ] Configure pytest and launch_testing
- [ ] Set up test fixtures and mock data
- [ ] Implement test data recording/playback

### Automated Testing
- [ ] Topic remapping test suite
  - [ ] LiDAR remapping validation
  - [ ] Camera remapping validation
  - [ ] IMU/GNSS remapping validation
- [ ] Sensor pose validation suite
  - [ ] Static transform tests
  - [ ] TF tree consistency tests
  - [ ] Dynamic transform tests
- [ ] Performance test suite
  - [ ] Latency measurements
  - [ ] Throughput validation
  - [ ] Resource usage monitoring

### CI/CD Integration
- [ ] Add GitHub Actions workflow
- [ ] Configure automated test execution
- [ ] Add test coverage reporting
- [ ] Create performance regression detection
- [ ] Document test procedures and guidelines

## Phase 8: Documentation & Training
**Goal**: Complete user documentation and examples

- [ ] Create user guide with screenshots
- [ ] Add troubleshooting guide
- [ ] Create video tutorials
- [ ] Implement example scenarios
- [ ] Add API documentation
- [ ] Create sensor configuration guide

## Phase 9: Community Release
**Goal**: Open source release preparation

- [ ] Code cleanup and refactoring
- [ ] License selection and headers
- [ ] Create contribution guidelines
- [ ] Set up issue templates
- [ ] Prepare release notes
- [ ] Create Docker images

## Long-term Goals

### Simulator Compatibility
- [ ] Support for CARLA 0.9.x versions
- [ ] Integration with other simulators (Gazebo, LGSVL)
- [ ] Unified sensor interface abstraction

### Autoware Integration
- [ ] Direct Autoware launcher integration
- [ ] Scenario runner compatibility
- [ ] Planning/control loop closure
- [ ] Full AD stack validation

### Cloud & Scale
- [ ] Cloud deployment support
- [ ] Distributed simulation
- [ ] Large-scale scenario testing
- [ ] Performance metrics collection

## Version Milestones

### v0.1.0 (Phase 1)
- Basic vehicle spawning
- Core sensor support
- Manual configuration

### v0.2.0 (Phase 2-3)
- Dynamic configuration system
- Vehicle interface
- Full sensor integration

### v0.3.0 (Phase 4-5)
- Calibration tools
- Multi-vehicle support
- Basic diagnostics

### v0.4.0 (Phase 6-7)
- Performance optimizations
- Advanced sensors
- Test framework

### v1.0.0 (Phase 8-9)
- Production ready
- Full documentation
- Community release

## Contributing

This roadmap is a living document. Contributions and suggestions are welcome through:
- GitHub issues for feature requests
- Pull requests for implementations
- Discussion forum for design decisions

## Dependencies

### External Projects
- CARLA 0.10.0 development
- Autoware.Universe updates
- ROS2 Humble maintenance

### Technical Debt
- Migration to newer CARLA versions
- ROS2 parameter server integration
- Launch file XML to Python migration