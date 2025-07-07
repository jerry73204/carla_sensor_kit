# CARLA Sensor Kit Launch - Development Roadmap

This document outlines the development phases for the CARLA sensor kit launch project, which integrates CARLA 0.10.0 simulator with Autoware through ROS2.

## Phase 1: Foundation (Current)
**Goal**: Establish basic vehicle spawning and sensor data flow

### Completed
- [x] Create project structure with ROS2 packages
- [x] Design documentation (ARCH.md, DESIGN.md)
- [x] Define CARLA ROS topic specifications
- [x] Design vehicle spawner script architecture

### In Progress
- [ ] Implement carla_spawner Python package
  - [ ] Create Rye project structure
  - [ ] Implement spawn_vehicle.py script
  - [ ] Add command-line argument parsing
  - [ ] Test with CARLA 0.10.0

### To Do
- [ ] Create sensor kit launch files
  - [ ] Implement topic remapping for each sensor type
  - [ ] Add launch parameters for actor names
  - [ ] Test data flow from CARLA to Autoware topics

## Phase 2: Sensor Integration
**Goal**: Full sensor suite working with accurate data transformation

### LiDAR Pipeline
- [ ] Implement pointcloud preprocessor launch
- [ ] Configure ring outlier filter
- [ ] Add distortion correction
- [ ] Validate coordinate frame transformations

### Camera Pipeline
- [ ] Enable camera launch file
- [ ] Configure image transport
- [ ] Add camera info publishing
- [ ] Test multiple camera positions

### IMU Integration
- [ ] Configure IMU corrector node
- [ ] Validate noise parameters
- [ ] Test data rates and latency

### GNSS Integration
- [ ] Configure GNSS poser node
- [ ] Implement coordinate conversions
- [ ] Add UTM zone handling

## Phase 3: Calibration & Transforms
**Goal**: Accurate sensor positioning and calibration

- [ ] Create sensor mounting configuration UI/tool
- [ ] Implement dynamic TF tree generation
- [ ] Add sensor calibration file generator
- [ ] Create calibration validation tools
- [ ] Document calibration procedures

## Phase 4: Multi-Vehicle Support
**Goal**: Support multiple vehicles in same simulation

- [ ] Extend spawner for multiple vehicles
- [ ] Implement namespace isolation for topics
- [ ] Add vehicle fleet configuration file
- [ ] Create traffic scenario scripts
- [ ] Test with 5+ simultaneous vehicles

## Phase 5: Advanced Features
**Goal**: Production-ready features and optimizations

### Performance Optimization
- [ ] Implement zero-copy message passing
- [ ] Add configurable data decimation
- [ ] Optimize pointcloud processing pipeline
- [ ] Profile and reduce latency

### Monitoring & Diagnostics
- [ ] Integrate diagnostic aggregator
- [ ] Add sensor health monitoring
- [ ] Create visualization dashboard
- [ ] Implement data recording tools

### Additional Sensors
- [ ] Add radar sensor support
- [ ] Implement ultrasonic sensors
- [ ] Add V2X communication simulation
- [ ] Support custom sensor types

## Phase 6: Testing & Validation
**Goal**: Comprehensive testing framework

- [ ] Create unit tests for spawner script
- [ ] Add integration tests for launch files
- [ ] Implement sensor data validation
- [ ] Create performance benchmarks
- [ ] Add CI/CD pipeline
- [ ] Document test procedures

## Phase 7: Documentation & Training
**Goal**: Complete user documentation and examples

- [ ] Create user guide with screenshots
- [ ] Add troubleshooting guide
- [ ] Create video tutorials
- [ ] Implement example scenarios
- [ ] Add API documentation
- [ ] Create sensor configuration guide

## Phase 8: Community Release
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

### v0.1.0 (Phase 1-2)
- Basic vehicle spawning
- Core sensor support
- Manual configuration

### v0.2.0 (Phase 3-4)
- Calibration tools
- Multi-vehicle support
- Basic diagnostics

### v0.3.0 (Phase 5-6)
- Performance optimizations
- Advanced sensors
- Test framework

### v1.0.0 (Phase 7-8)
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