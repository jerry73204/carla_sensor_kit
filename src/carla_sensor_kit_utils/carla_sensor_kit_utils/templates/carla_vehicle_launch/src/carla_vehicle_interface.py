#!/usr/bin/env python3
"""
CARLA Vehicle Interface Node

This ROS2 node provides the interface between Autoware control commands
and CARLA vehicle control, including state publishing and dynamic configuration.
"""

import math
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor

# Autoware messages
from autoware_auto_control_msgs.msg import AckermannControlCommand
from autoware_auto_vehicle_msgs.msg import (
    ControlModeReport,
    GearCommand,
    GearReport,
    HazardLightsCommand,
    HazardLightsReport,
    SteeringReport,
    TurnIndicatorsCommand,
    TurnIndicatorsReport,
    VelocityReport
)
from geometry_msgs.msg import TwistStamped

try:
    import carla
except ImportError:
    print("Warning: CARLA Python API not found. Some features may be limited.")
    carla = None


class CARLAVehicleInterface(Node):
    """ROS2 node for CARLA vehicle interface"""
    
    def __init__(self):
        """Initialize the CARLA vehicle interface node"""
        super().__init__('carla_vehicle_interface')
        
        # Declare parameters
        self._declare_parameters()
        
        # CARLA connection
        self.carla_client = None
        self.carla_world = None
        self.vehicle_actor = None
        self._connection_lock = threading.Lock()
        
        # Vehicle parameters (will be loaded from ROS parameters)
        self.max_steer_angle = self.get_parameter('max_steer_angle').value
        self.max_acceleration = self.get_parameter('max_acceleration').value
        self.max_deceleration = self.get_parameter('max_deceleration').value
        
        # Control state
        self.current_control = carla.VehicleControl() if carla else None
        self.control_mode = ControlModeReport.AUTONOMOUS
        self.gear_state = GearReport.DRIVE
        self.turn_indicators = TurnIndicatorsReport.DISABLE
        self.hazard_lights = False
        
        # Initialize CARLA connection
        self._init_carla_connection()
        
        # Create subscribers
        self._create_subscribers()
        
        # Create publishers
        self._create_publishers()
        
        # Create timers
        self.state_timer = self.create_timer(
            1.0 / self.get_parameter('state_publish_rate').value,
            self.publish_vehicle_state
        )
        
        self.connection_timer = self.create_timer(
            5.0,  # Check connection every 5 seconds
            self._check_connection
        )
        
        self.get_logger().info('CARLA Vehicle Interface initialized')
    
    def _declare_parameters(self):
        """Declare ROS parameters"""
        # CARLA connection parameters
        self.declare_parameter('carla_host', 'localhost',
                             ParameterDescriptor(description='CARLA server host'))
        self.declare_parameter('carla_port', 2000,
                             ParameterDescriptor(description='CARLA server port'))
        self.declare_parameter('vehicle_role_name', 'ego_vehicle',
                             ParameterDescriptor(description='Vehicle role name in CARLA'))
        
        # Vehicle parameters
        self.declare_parameter('max_steer_angle', 0.7,
                             ParameterDescriptor(description='Maximum steering angle in radians'))
        self.declare_parameter('max_acceleration', 3.0,
                             ParameterDescriptor(description='Maximum acceleration in m/s^2'))
        self.declare_parameter('max_deceleration', 8.0,
                             ParameterDescriptor(description='Maximum deceleration in m/s^2'))
        
        # Update rates
        self.declare_parameter('control_update_rate', 50.0,
                             ParameterDescriptor(description='Control command update rate in Hz'))
        self.declare_parameter('state_publish_rate', 50.0,
                             ParameterDescriptor(description='Vehicle state publish rate in Hz'))
        
        # Timeouts
        self.declare_parameter('connection_timeout', 10.0,
                             ParameterDescriptor(description='CARLA connection timeout in seconds'))
        self.declare_parameter('control_timeout', 0.5,
                             ParameterDescriptor(description='Control command timeout in seconds'))
    
    def _init_carla_connection(self):
        """Initialize connection to CARLA"""
        if not carla:
            self.get_logger().error('CARLA Python API not available')
            return
        
        try:
            # Connect to CARLA
            host = self.get_parameter('carla_host').value
            port = self.get_parameter('carla_port').value
            timeout = self.get_parameter('connection_timeout').value
            
            self.get_logger().info(f'Connecting to CARLA at {host}:{port}')
            
            self.carla_client = carla.Client(host, port)
            self.carla_client.set_timeout(timeout)
            
            # Get world
            self.carla_world = self.carla_client.get_world()
            
            # Find vehicle by role name
            self._find_vehicle()
            
        except Exception as e:
            self.get_logger().error(f'Failed to connect to CARLA: {e}')
            self.carla_client = None
            self.carla_world = None
    
    def _find_vehicle(self):
        """Find vehicle actor by role name"""
        if not self.carla_world:
            return
        
        role_name = self.get_parameter('vehicle_role_name').value
        actors = self.carla_world.get_actors().filter('vehicle.*')
        
        for actor in actors:
            if actor.attributes.get('role_name') == role_name:
                self.vehicle_actor = actor
                self.get_logger().info(f'Found vehicle with role_name: {role_name}')
                
                # Extract and log vehicle info
                self._log_vehicle_info()
                return
        
        self.get_logger().warning(f'Vehicle with role_name "{role_name}" not found')
    
    def _log_vehicle_info(self):
        """Log vehicle information"""
        if not self.vehicle_actor:
            return
        
        try:
            physics = self.vehicle_actor.get_physics_control()
            bbox = self.vehicle_actor.bounding_box
            
            self.get_logger().info(
                f'Vehicle info - Type: {self.vehicle_actor.type_id}, '
                f'Mass: {physics.mass}kg, '
                f'Dimensions: {bbox.extent.x*2:.1f}x{bbox.extent.y*2:.1f}x{bbox.extent.z*2:.1f}cm'
            )
        except Exception as e:
            self.get_logger().warning(f'Failed to get vehicle info: {e}')
    
    def _create_subscribers(self):
        """Create ROS subscribers"""
        # Control command subscriber
        self.control_cmd_sub = self.create_subscription(
            AckermannControlCommand,
            '/control/command/control_cmd',
            self.control_callback,
            QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                depth=1
            )
        )
        
        # Gear command subscriber
        self.gear_cmd_sub = self.create_subscription(
            GearCommand,
            '/control/command/gear_cmd',
            self.gear_callback,
            10
        )
        
        # Turn indicators command subscriber
        self.turn_indicators_cmd_sub = self.create_subscription(
            TurnIndicatorsCommand,
            '/control/command/turn_indicators_cmd',
            self.turn_indicators_callback,
            10
        )
        
        # Hazard lights command subscriber
        self.hazard_lights_cmd_sub = self.create_subscription(
            HazardLightsCommand,
            '/control/command/hazard_lights_cmd',
            self.hazard_lights_callback,
            10
        )
    
    def _create_publishers(self):
        """Create ROS publishers"""
        # QoS profile for vehicle status
        status_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        
        # Velocity status publisher
        self.velocity_pub = self.create_publisher(
            VelocityReport,
            '/vehicle/status/velocity_status',
            status_qos
        )
        
        # Steering status publisher
        self.steering_pub = self.create_publisher(
            SteeringReport,
            '/vehicle/status/steering_status',
            status_qos
        )
        
        # Gear status publisher
        self.gear_pub = self.create_publisher(
            GearReport,
            '/vehicle/status/gear_status',
            status_qos
        )
        
        # Control mode publisher
        self.control_mode_pub = self.create_publisher(
            ControlModeReport,
            '/vehicle/status/control_mode',
            status_qos
        )
        
        # Turn indicators status publisher
        self.turn_indicators_pub = self.create_publisher(
            TurnIndicatorsReport,
            '/vehicle/status/turn_indicators_status',
            status_qos
        )
        
        # Hazard lights status publisher
        self.hazard_lights_pub = self.create_publisher(
            HazardLightsReport,
            '/vehicle/status/hazard_lights_status',
            status_qos
        )
        
        # Twist publisher (for compatibility)
        self.twist_pub = self.create_publisher(
            TwistStamped,
            '/vehicle/status/twist',
            10
        )
    
    def control_callback(self, msg: AckermannControlCommand):
        """
        Handle control command from Autoware.
        
        Args:
            msg: AckermannControlCommand message
        """
        if not self.vehicle_actor or not carla:
            return
        
        with self._connection_lock:
            try:
                # Create CARLA control
                control = carla.VehicleControl()
                
                # Map steering
                # Autoware: front wheel angle in radians
                # CARLA: normalized steering [-1, 1]
                control.steer = msg.lateral.steering_tire_angle / self.max_steer_angle
                control.steer = max(-1.0, min(1.0, control.steer))
                
                # Map throttle/brake based on acceleration
                target_accel = msg.longitudinal.acceleration
                
                if target_accel >= 0:
                    # Positive acceleration - throttle
                    control.throttle = min(target_accel / self.max_acceleration, 1.0)
                    control.brake = 0.0
                else:
                    # Negative acceleration - brake
                    control.throttle = 0.0
                    control.brake = min(-target_accel / self.max_deceleration, 1.0)
                
                # Set gear and other states
                control.hand_brake = False
                control.reverse = (self.gear_state == GearReport.REVERSE)
                control.manual_gear_shift = False
                control.gear = 0
                
                # Apply control
                self.vehicle_actor.apply_control(control)
                self.current_control = control
                
            except Exception as e:
                self.get_logger().error(f'Failed to apply control: {e}')
    
    def gear_callback(self, msg: GearCommand):
        """
        Handle gear command.
        
        Args:
            msg: GearCommand message
        """
        # Map Autoware gear to internal state
        if msg.command == GearCommand.PARK:
            self.gear_state = GearReport.PARK
        elif msg.command == GearCommand.REVERSE:
            self.gear_state = GearReport.REVERSE
        elif msg.command == GearCommand.NEUTRAL:
            self.gear_state = GearReport.NEUTRAL
        elif msg.command == GearCommand.DRIVE:
            self.gear_state = GearReport.DRIVE
        # LOW and specific drive gears map to DRIVE
        else:
            self.gear_state = GearReport.DRIVE
    
    def turn_indicators_callback(self, msg: TurnIndicatorsCommand):
        """
        Handle turn indicators command.
        
        Args:
            msg: TurnIndicatorsCommand message
        """
        self.turn_indicators = msg.command
        
        # Apply to CARLA vehicle if available
        if self.vehicle_actor and hasattr(self.vehicle_actor, 'set_light_state'):
            try:
                light_state = carla.VehicleLightState.NONE
                
                if msg.command == TurnIndicatorsCommand.ENABLE_LEFT:
                    light_state |= carla.VehicleLightState.LeftBlinker
                elif msg.command == TurnIndicatorsCommand.ENABLE_RIGHT:
                    light_state |= carla.VehicleLightState.RightBlinker
                
                if self.hazard_lights:
                    light_state |= carla.VehicleLightState.LeftBlinker
                    light_state |= carla.VehicleLightState.RightBlinker
                
                self.vehicle_actor.set_light_state(carla.VehicleLightState(light_state))
            except Exception as e:
                self.get_logger().debug(f'Failed to set turn indicators: {e}')
    
    def hazard_lights_callback(self, msg: HazardLightsCommand):
        """
        Handle hazard lights command.
        
        Args:
            msg: HazardLightsCommand message
        """
        self.hazard_lights = (msg.command == HazardLightsCommand.ENABLE)
        
        # Update lights
        self.turn_indicators_callback(
            TurnIndicatorsCommand(command=self.turn_indicators)
        )
    
    def publish_vehicle_state(self):
        """Query CARLA and publish vehicle state"""
        if not self.vehicle_actor:
            # Publish disconnected state
            self._publish_disconnected_state()
            return
        
        with self._connection_lock:
            try:
                # Get vehicle state from CARLA
                velocity = self.vehicle_actor.get_velocity()
                transform = self.vehicle_actor.get_transform()
                control = self.vehicle_actor.get_control()
                
                # Calculate speeds
                speed_mps = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2) / 100.0  # cm/s to m/s
                
                # Transform velocity to vehicle frame
                yaw = math.radians(transform.rotation.yaw)
                vx_world = velocity.x / 100.0
                vy_world = velocity.y / 100.0
                
                vx_vehicle = vx_world * math.cos(yaw) + vy_world * math.sin(yaw)
                vy_vehicle = -vx_world * math.sin(yaw) + vy_world * math.cos(yaw)
                
                # Calculate heading rate (yaw rate)
                # This is simplified - ideally would use angular velocity
                heading_rate = (vx_vehicle * math.tan(control.steer * self.max_steer_angle)) / 2.5  # Assume 2.5m wheelbase
                
                # Publish velocity report
                vel_msg = VelocityReport()
                vel_msg.header.stamp = self.get_clock().now().to_msg()
                vel_msg.header.frame_id = "base_link"
                vel_msg.longitudinal_velocity = vx_vehicle
                vel_msg.lateral_velocity = -vy_vehicle  # ROS convention
                vel_msg.heading_rate = heading_rate
                self.velocity_pub.publish(vel_msg)
                
                # Publish steering report
                steer_msg = SteeringReport()
                steer_msg.stamp = self.get_clock().now().to_msg()
                steer_msg.steering_tire_angle = control.steer * self.max_steer_angle
                self.steering_pub.publish(steer_msg)
                
                # Publish gear report
                gear_msg = GearReport()
                gear_msg.stamp = self.get_clock().now().to_msg()
                gear_msg.report = self.gear_state
                self.gear_pub.publish(gear_msg)
                
                # Publish control mode
                mode_msg = ControlModeReport()
                mode_msg.stamp = self.get_clock().now().to_msg()
                mode_msg.mode = self.control_mode
                self.control_mode_pub.publish(mode_msg)
                
                # Publish turn indicators
                turn_msg = TurnIndicatorsReport()
                turn_msg.stamp = self.get_clock().now().to_msg()
                turn_msg.report = self.turn_indicators
                self.turn_indicators_pub.publish(turn_msg)
                
                # Publish hazard lights
                hazard_msg = HazardLightsReport()
                hazard_msg.stamp = self.get_clock().now().to_msg()
                hazard_msg.report = HazardLightsReport.ENABLE if self.hazard_lights else HazardLightsReport.DISABLE
                self.hazard_lights_pub.publish(hazard_msg)
                
                # Publish twist for compatibility
                twist_msg = TwistStamped()
                twist_msg.header.stamp = self.get_clock().now().to_msg()
                twist_msg.header.frame_id = "base_link"
                twist_msg.twist.linear.x = vx_vehicle
                twist_msg.twist.linear.y = vy_vehicle
                twist_msg.twist.linear.z = velocity.z / 100.0
                twist_msg.twist.angular.z = heading_rate
                self.twist_pub.publish(twist_msg)
                
            except Exception as e:
                self.get_logger().error(f'Failed to publish vehicle state: {e}')
                self._publish_disconnected_state()
    
    def _publish_disconnected_state(self):
        """Publish state when disconnected from CARLA"""
        # Publish control mode as NO_COMMAND
        mode_msg = ControlModeReport()
        mode_msg.stamp = self.get_clock().now().to_msg()
        mode_msg.mode = ControlModeReport.NO_COMMAND
        self.control_mode_pub.publish(mode_msg)
        
        # Publish zero velocity
        vel_msg = VelocityReport()
        vel_msg.header.stamp = self.get_clock().now().to_msg()
        vel_msg.header.frame_id = "base_link"
        vel_msg.longitudinal_velocity = 0.0
        vel_msg.lateral_velocity = 0.0
        vel_msg.heading_rate = 0.0
        self.velocity_pub.publish(vel_msg)
    
    def _check_connection(self):
        """Periodically check CARLA connection"""
        if not self.vehicle_actor:
            self.get_logger().debug('Attempting to reconnect to CARLA vehicle...')
            self._find_vehicle()
    
    def destroy_node(self):
        """Clean up on node shutdown"""
        # Set vehicle to neutral/stopped state
        if self.vehicle_actor and carla:
            try:
                stop_control = carla.VehicleControl()
                stop_control.throttle = 0.0
                stop_control.brake = 1.0
                stop_control.steer = 0.0
                self.vehicle_actor.apply_control(stop_control)
            except:
                pass
        
        super().destroy_node()


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    try:
        node = CARLAVehicleInterface()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()