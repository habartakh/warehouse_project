import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time_arg =  launch.actions.DeclareLaunchArgument('use_sim_time', default_value='False')
    use_sim_time_f = LaunchConfiguration('use_sim_time')

    # For the simulation
    if use_sim_time_f == 'true':
        map_file_name = 'warehouse_map_sim.yaml'
    #For the real robot 
    elif use_sim_time_f == 'false':
        map_file_name = 'warehouse_map_real'
    
    map_file = os.path.join(get_package_share_directory('map_server'), 'config', map_file_name)
    rviz_config_dir = os.path.join(get_package_share_directory('cartographer_slam'), 'rviz', 'map_display.rviz')


    return LaunchDescription([
        use_sim_time_arg,
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            name='rviz_node',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', rviz_config_dir]),


        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'yaml_filename':map_file}
                       ]),


        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_mapper',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': ['map_server']}]),

                
        ])
