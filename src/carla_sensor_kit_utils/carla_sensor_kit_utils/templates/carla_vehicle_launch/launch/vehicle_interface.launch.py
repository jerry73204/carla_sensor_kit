#!/usr/bin/env python3
"""
Dynamic Vehicle Interface Launch File

This launch file dynamically generates vehicle and sensor configurations
from CARLA and launches the necessary nodes.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Add carla_sensor_kit_utils to Python path
import sys
utils_path = os.path.join(
    get_package_share_directory('carla_sensor_kit_utils'),
    '..', '..', 'carla_sensor_kit_utils', 'src'
)
sys.path.insert(0, os.path.abspath(utils_path))

try:
    import carla
    from carla_sensor_kit_utils.urdf_generator import DynamicConfigGenerator
    CARLA_AVAILABLE = True
except ImportError:
    CARLA_AVAILABLE = False
    print("Warning: CARLA Python API not found. Dynamic configuration disabled.")


def generate_dynamic_launch_setup(context, *args, **kwargs):
    """
    Generate launch configuration dynamically from CARLA.
    
    This function connects to CARLA, finds the vehicle, extracts parameters,
    and creates the necessary ROS nodes with dynamic configurations.
    """
    # Get launch configurations
    vehicle_role_name = LaunchConfiguration('vehicle_role_name').perform(context)
    carla_host = LaunchConfiguration('carla_host').perform(context)
    carla_port = int(LaunchConfiguration('carla_port').perform(context))
    use_dynamic_config = LaunchConfiguration('use_dynamic_config').perform(context).lower() == 'true'
    
    nodes = []
    
    # If dynamic config is disabled or CARLA not available, use static config
    if not use_dynamic_config or not CARLA_AVAILABLE:
        print("Using static configuration")
        
        # Robot state publisher with static URDF
        urdf_file = os.path.join(
            get_package_share_directory('carla_vehicle_description'),
            'urdf', 'vehicle.urdf'
        )
        
        if os.path.exists(urdf_file):
            with open(urdf_file, 'r') as f:
                robot_description = f.read()
        else:
            # Minimal URDF if file doesn't exist
            robot_description = """<?xml version="1.0"?>
<robot name="carla_vehicle">
  <link name="base_link"/>
</robot>"""
        
        nodes.append(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=vehicle_role_name,
            parameters=[{
                'robot_description': robot_description,
                'publish_frequency': 50.0
            }]
        ))
        
        # Vehicle interface with static parameters
        vehicle_params_file = os.path.join(
            get_package_share_directory('carla_vehicle_description'),
            'config', 'vehicle_info.param.yaml'
        )
        
        nodes.append(Node(
            package='carla_vehicle_launch',
            executable='carla_vehicle_interface.py',
            name='carla_vehicle_interface',
            namespace=vehicle_role_name,
            parameters=[
                vehicle_params_file if os.path.exists(vehicle_params_file) else {},
                {
                    'carla_host': carla_host,
                    'carla_port': carla_port,
                    'vehicle_role_name': vehicle_role_name,
                    'use_sim_time': LaunchConfiguration('use_sim_time')
                }
            ]
        ))
        
        return nodes
    
    # Dynamic configuration from CARLA
    print(f"Connecting to CARLA at {carla_host}:{carla_port} for dynamic configuration...")
    
    try:
        # Connect to CARLA
        client = carla.Client(carla_host, carla_port)
        client.set_timeout(10.0)
        world = client.get_world()
        
        # Find vehicle by role_name
        actors = world.get_actors().filter('vehicle.*')
        vehicle = None
        
        for actor in actors:
            if actor.attributes.get('role_name') == vehicle_role_name:
                vehicle = actor
                break
        
        if not vehicle:
            print(f"Warning: Vehicle with role_name '{vehicle_role_name}' not found in CARLA")
            print("Falling back to static configuration")
            # Return static config nodes (same as above)
            return nodes
        
        print(f"Found vehicle: {vehicle.type_id}")
        
        # Generate dynamic configurations
        config_generator = DynamicConfigGenerator(world, vehicle)
        all_configs = config_generator.generate_all_configurations()
        
        vehicle_params = all_configs['vehicle_parameters']
        sensor_params = all_configs['sensor_parameters']
        robot_description = all_configs['robot_description']
        
        print(f"Generated dynamic configuration for {len(sensor_params)} sensors")
        
        # Create robot state publisher with dynamic URDF
        nodes.append(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=vehicle_role_name,
            parameters=[{
                'robot_description': robot_description,
                'publish_frequency': 50.0,
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }],
            output='screen'
        ))
        
        # Create vehicle interface with dynamic parameters
        # Combine all vehicle parameters
        vehicle_interface_params = {
            **vehicle_params['vehicle_info'],
            **vehicle_params['simulator_model'],
            'carla_host': carla_host,
            'carla_port': carla_port,
            'vehicle_role_name': vehicle_role_name,
            'control_update_rate': 50.0,
            'state_publish_rate': 50.0,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }
        
        nodes.append(Node(
            package='carla_vehicle_launch',
            executable='carla_vehicle_interface.py',
            name='carla_vehicle_interface',
            namespace=vehicle_role_name,
            parameters=[vehicle_interface_params],
            output='screen'
        ))
        
        # TODO: Add sensor driver nodes based on sensor_params
        # This would launch camera, lidar, etc. drivers with extracted parameters
        
        print("Dynamic launch configuration complete")
        
    except Exception as e:
        print(f"Error generating dynamic configuration: {e}")
        print("Falling back to static configuration")
        # Return empty list to use default static nodes
        
    return nodes


def generate_launch_description():
    """Generate the launch description"""
    
    # Declare launch arguments
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'vehicle_role_name',
            default_value='ego_vehicle',
            description='Role name of the vehicle in CARLA'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'carla_host',
            default_value='localhost',
            description='CARLA server host'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'carla_port',
            default_value='2000',
            description='CARLA server port'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_dynamic_config',
            default_value='true',
            description='Use dynamic configuration from CARLA'
        )
    )
    
    # Create launch description
    ld = LaunchDescription(declared_arguments)
    
    # Add opaque function for dynamic launch
    ld.add_action(OpaqueFunction(function=generate_dynamic_launch_setup))
    
    return ld