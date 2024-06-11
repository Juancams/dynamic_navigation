import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    map_yaml_file = LaunchConfiguration('map')
    long_map_yaml_file = LaunchConfiguration('long_map')

    map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(
            get_package_share_directory('dynamic_navigation'),
            'maps',
            'lab.yaml'
        ),
        description='Full path to map file to load'
    )

    long_map_yaml_cmd = DeclareLaunchArgument(
        'long_map',
        default_value=os.path.join(
            get_package_share_directory('dynamic_navigation'),
            'maps',
            'lab_long.yaml'
        ),
        description='Full path to long_map file to load'
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_yaml_file}],
        remappings=[('/map', '/static_map')]
    )

    long_map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='long_map_server',
        output='screen',
        parameters=[{'yaml_filename': long_map_yaml_file}],
        remappings=[('/map', '/long_map')]
    )

    map_saver_node = Node(
        package='dynamic_navigation',
        executable='map_saver_node',
        name='map_saver',
        output='screen',
        parameters=[{'save_map_path': os.path.join(get_package_share_directory('dynamic_navigation'), 'maps/')},
                    {'yaml_filename': long_map_yaml_file}]
                    
    )

    ld = LaunchDescription()
    ld.add_action(map_yaml_cmd)
    ld.add_action(long_map_yaml_cmd)
    ld.add_action(map_server_node)
    ld.add_action(long_map_server_node)
    ld.add_action(map_saver_node)

    return ld