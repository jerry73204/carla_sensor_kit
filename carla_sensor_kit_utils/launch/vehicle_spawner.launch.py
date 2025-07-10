"""Launch file for CARLA vehicle spawner node."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    host_arg = DeclareLaunchArgument(
        'host',
        default_value='localhost',
        description='CARLA server host'
    )
    
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='2000',
        description='CARLA server port'
    )
    
    vehicle_type_arg = DeclareLaunchArgument(
        'vehicle_type',
        default_value='vehicle.tesla.model3',
        description='Vehicle blueprint to spawn'
    )
    
    vehicle_role_name_arg = DeclareLaunchArgument(
        'vehicle_role_name',
        default_value='hero',
        description='Vehicle role name for identification'
    )
    
    spawn_point_arg = DeclareLaunchArgument(
        'spawn_point_index',
        default_value='-1',
        description='Spawn point index (-1 for random)'
    )
    
    autopilot_arg = DeclareLaunchArgument(
        'autopilot',
        default_value='True',
        description='Enable autopilot for spawned vehicle'
    )
    
    enable_sensors_arg = DeclareLaunchArgument(
        'enable_sensors',
        default_value='True',
        description='Enable sensor spawning on vehicle'
    )
    
    # Create spawner node
    spawner_node = Node(
        package='carla_sensor_kit_utils',
        executable='vehicle_spawner',
        name='carla_vehicle_spawner',
        output='screen',
        parameters=[{
            'host': LaunchConfiguration('host'),
            'port': LaunchConfiguration('port'),
            'vehicle_type': LaunchConfiguration('vehicle_type'),
            'vehicle_role_name': LaunchConfiguration('vehicle_role_name'),
            'spawn_point_index': LaunchConfiguration('spawn_point_index'),
            'autopilot': LaunchConfiguration('autopilot'),
            'enable_sensors': LaunchConfiguration('enable_sensors'),
        }]
    )
    
    return LaunchDescription([
        host_arg,
        port_arg,
        vehicle_type_arg,
        vehicle_role_name_arg,
        spawn_point_arg,
        autopilot_arg,
        enable_sensors_arg,
        spawner_node
    ])