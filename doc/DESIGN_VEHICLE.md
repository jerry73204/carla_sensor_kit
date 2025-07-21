# CARLA Vehicle Interface Detailed Design

## Overview

This document provides the detailed design for the CARLA vehicle interface system, including control bridging, state synchronization, and dynamic parameter generation.

## Vehicle Interface Components

### 1. CARLA Vehicle Interface Node

The main ROS2 node that bridges Autoware control commands to CARLA vehicle control.

#### Class Structure

```python
class CARLAVehicleInterface(Node):
    """ROS2 node for CARLA vehicle interface"""

    def __init__(self):
        super().__init__('carla_vehicle_interface')

        # CARLA connection
        self.carla_client = None
        self.carla_world = None
        self.vehicle_actor = None

        # Control subscribers
        self.control_cmd_sub = self.create_subscription(
            AckermannControlCommand,
            '/control/command/control_cmd',
            self.control_callback,
            1
        )

        # Status publishers
        self.velocity_pub = self.create_publisher(
            VelocityReport,
            '/vehicle/status/velocity_status',
            1
        )

        # State update timer
        self.state_timer = self.create_timer(
            0.02,  # 50Hz
            self.publish_vehicle_state
        )
```

#### Control Command Translation

```python
def control_callback(self, msg: AckermannControlCommand):
    """Translate Autoware control to CARLA"""
    if not self.vehicle_actor:
        return

    # Create CARLA control
    control = carla.VehicleControl()

    # Map steering (Autoware uses front wheel angle)
    control.steer = msg.lateral.steering_tire_angle / self.max_steer_angle

    # Map throttle/brake
    if msg.longitudinal.acceleration >= 0:
        control.throttle = min(msg.longitudinal.acceleration / self.max_acceleration, 1.0)
        control.brake = 0.0
    else:
        control.throttle = 0.0
        control.brake = min(-msg.longitudinal.acceleration / self.max_deceleration, 1.0)

    # Apply control
    self.vehicle_actor.apply_control(control)
```

#### State Publishing

```python
def publish_vehicle_state(self):
    """Query CARLA and publish vehicle state"""
    if not self.vehicle_actor:
        return

    # Get velocity
    velocity = self.vehicle_actor.get_velocity()
    speed = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)

    # Get steering
    control = self.vehicle_actor.get_control()
    steering_angle = control.steer * self.max_steer_angle

    # Publish velocity report
    vel_msg = VelocityReport()
    vel_msg.header.stamp = self.get_clock().now().to_msg()
    vel_msg.header.frame_id = "base_link"
    vel_msg.longitudinal_velocity = speed
    vel_msg.lateral_velocity = 0.0  # Simplified
    vel_msg.heading_rate = self.calculate_heading_rate(velocity, steering_angle)
    self.velocity_pub.publish(vel_msg)
```

### 2. Parameter Extraction System

#### VehicleParameterExtractor Class

```python
class VehicleParameterExtractor:
    """Extracts vehicle parameters from CARLA actor"""

    def __init__(self, vehicle_actor: carla.Vehicle):
        self.vehicle = vehicle_actor
        self._physics_control = None
        self._bounding_box = None

    def extract_wheel_parameters(self) -> Dict[str, Any]:
        """Extract wheel-related parameters"""
        wheels = self.physics_control.wheels

        # Find wheel positions
        front_wheels = [w for w in wheels if w.position.x > 0]
        rear_wheels = [w for w in wheels if w.position.x <= 0]
        left_wheels = [w for w in wheels if w.position.y > 0]
        right_wheels = [w for w in wheels if w.position.y <= 0]

        # Calculate parameters (convert cm to m)
        wheel_base = abs(front_wheels[0].position.x - rear_wheels[0].position.x) / 100.0
        wheel_tread = abs(left_wheels[0].position.y - right_wheels[0].position.y) / 100.0

        return {
            'wheel_radius': wheels[0].radius / 100.0,
            'wheel_width': 0.235,  # Default, CARLA doesn't provide
            'wheel_base': wheel_base,
            'wheel_tread': wheel_tread,
            'max_steer_angle': max(w.max_steer_angle for w in front_wheels) * (np.pi / 180.0)
        }

    def extract_vehicle_dimensions(self) -> Dict[str, float]:
        """Extract vehicle dimension parameters"""
        extent = self.bounding_box.extent

        # Convert from cm to m
        length = extent.x * 2.0 / 100.0
        width = extent.y * 2.0 / 100.0
        height = extent.z * 2.0 / 100.0

        # Calculate overhangs
        wheel_params = self.extract_wheel_parameters()
        wheels = self.physics_control.wheels

        front_wheel_x = max(w.position.x for w in wheels) / 100.0
        rear_wheel_x = min(w.position.x for w in wheels) / 100.0

        front_overhang = (length / 2.0) - front_wheel_x
        rear_overhang = (length / 2.0) + abs(rear_wheel_x)

        # Lateral overhangs
        left_overhang = (width / 2.0 - wheel_params['wheel_tread'] / 2.0) / 2.0
        right_overhang = left_overhang

        return {
            'vehicle_length': length,
            'vehicle_width': width,
            'vehicle_height': height,
            'front_overhang': front_overhang,
            'rear_overhang': rear_overhang,
            'left_overhang': left_overhang,
            'right_overhang': right_overhang
        }
```

#### SensorParameterExtractor Class

```python
class SensorParameterExtractor:
    """Extracts sensor parameters from CARLA sensor actors"""

    def __init__(self, world: carla.World, vehicle_actor: carla.Vehicle):
        self.world = world
        self.vehicle = vehicle_actor

    def extract_sensor_transform(self, sensor: carla.Actor) -> Dict[str, float]:
        """Extract sensor position and orientation relative to vehicle"""
        transform = sensor.get_transform()
        vehicle_transform = self.vehicle.get_transform()

        # Calculate relative transform
        relative_location = transform.location - vehicle_transform.location
        relative_location = vehicle_transform.rotation.get_inverse_matrix().transform_point(relative_location)

        # Convert from CARLA to ROS coordinate system
        return {
            'x': relative_location.x / 100.0,  # cm to m
            'y': -relative_location.y / 100.0,  # flip Y axis
            'z': relative_location.z / 100.0,
            'roll': math.radians(transform.rotation.roll),
            'pitch': math.radians(-transform.rotation.pitch),
            'yaw': math.radians(-transform.rotation.yaw)
        }
```

### 3. Dynamic URDF Generation

```python
class DynamicURDFGenerator:
    """Generates URDF from CARLA parameters"""

    def generate_urdf_string(self, vehicle_params: Dict, sensor_params: Dict) -> str:
        """Generate complete URDF as string"""
        urdf = self._generate_header()
        urdf += self._generate_base_link(vehicle_params)
        urdf += self._generate_sensor_kit_link()

        for sensor_name, sensor_config in sensor_params.items():
            urdf += self._generate_sensor_link(sensor_name, sensor_config)

        urdf += '</robot>'
        return urdf

    def _generate_base_link(self, params: Dict) -> str:
        """Generate base_link with vehicle dimensions"""
        return f'''
  <link name="base_link">
    <visual>
      <origin xyz="{params['wheel_base']/2} 0 {params['vehicle_height']/2}" rpy="0 0 0"/>
      <geometry>
        <box size="{params['vehicle_length']} {params['vehicle_width']} {params['vehicle_height']}"/>
      </geometry>
    </visual>
  </link>
'''
```

### 4. Launch Integration

```python
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node

def generate_dynamic_launch_setup(context, *args, **kwargs):
    """Generate launch configuration dynamically from CARLA"""

    # Connect to CARLA
    client = carla.Client('localhost', 2000)
    world = client.get_world()

    # Find vehicle by role_name
    vehicle_role = LaunchConfiguration('vehicle_role_name').perform(context)
    vehicle = find_vehicle_by_role(world, vehicle_role)

    # Generate configurations
    config_gen = DynamicConfigGenerator(world, vehicle)
    vehicle_params = config_gen.generate_vehicle_parameters()
    robot_description = config_gen.generate_urdf_string()

    # Create nodes
    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=vehicle_role,
            parameters=[{'robot_description': robot_description}]
        ),
        Node(
            package='carla_vehicle_launch',
            executable='carla_vehicle_interface',
            namespace=vehicle_role,
            parameters=[{
                **vehicle_params['vehicle_info'],
                'carla_host': 'localhost',
                'carla_port': 2000,
                'vehicle_role_name': vehicle_role
            }]
        )
    ]
```

## Design Decisions

### 1. Dynamic Configuration vs Static Files

**Decision**: Generate all configurations at runtime without files

**Rationale**:
- Works in read-only environments
- Always matches actual CARLA state
- Supports any vehicle blueprint
- No file management overhead

### 2. Direct CARLA API vs Bridge

**Decision**: Use CARLA Python API directly

**Rationale**:
- Lower latency
- More control over data flow
- Simpler architecture
- Better error handling

### 3. Polling vs Callbacks

**Decision**: Timer-based state polling

**Rationale**:
- Predictable timing for control systems
- Easier to debug
- Matches Autoware patterns
- Configurable update rates

### 4. Coordinate System Handling

**Decision**: Transform at interface level

**Rationale**:
- Clear transformation boundary
- Non-invasive to CARLA
- Easy to verify
- Standard approach

## Error Handling

### Connection Loss
```python
def handle_carla_error(self, error):
    """Handle CARLA connection errors"""
    self.get_logger().error(f"CARLA error: {error}")

    # Publish degraded status
    self.publish_control_mode(ControlModeReport.NO_COMMAND)

    # Attempt reconnection
    self.schedule_reconnection()
```

### Invalid Parameters
```python
def validate_parameters(self, params: Dict) -> bool:
    """Validate extracted parameters"""
    checks = [
        0.1 <= params.get('wheel_radius', 0) <= 1.0,
        0.5 <= params.get('wheel_base', 0) <= 10.0,
        params.get('mass', 0) > 0
    ]
    return all(checks)
```

## Performance Considerations

1. **Update Rates**:
   - Control commands: As received (typically 50Hz)
   - State publishing: Configurable (default 50Hz)
   - Parameter extraction: Once at startup

2. **Optimization**:
   - Cache physics control data
   - Batch state queries
   - Use composable nodes where possible

3. **Resource Usage**:
   - Minimal memory (no file I/O)
   - CPU usage scales with update rate
   - Network bandwidth for CARLA API

## Testing Strategy

1. **Unit Tests**:
   - Parameter extraction accuracy
   - Coordinate transformations
   - Control command mapping

2. **Integration Tests**:
   - CARLA connection handling
   - Multi-vehicle scenarios
   - Error recovery

3. **System Tests**:
   - Full Autoware integration
   - Performance benchmarks
   - Long-duration stability
