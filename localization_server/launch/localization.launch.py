import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Name of the current package
    package_description = 'localization_server'

    # Name of the file containing the map and its path
    map_file_arg =  launch.actions.DeclareLaunchArgument('map_file', default_value='warehouse_map_sim.yaml')
    map_file_f = LaunchConfiguration('map_file')

    map_file = PathJoinSubstitution([
        get_package_share_directory('map_server'),
        'config',
        map_file_f
    ])    
    
    # The path to the config file containing the parameters for the AMCL algorithm
    amcl_config_sim = "amcl_config_sim.yaml"
    amcl_config_real = "amcl_config_real.yaml"

   # Use PythonExpression to choose the full YAML path
    nav2_yaml = PythonExpression([
        '"', os.path.join(get_package_share_directory(package_description), 'config', amcl_config_sim), 
        '" if "', map_file_f, '" == "warehouse_map_keepout_sim.yaml" else "', 
        os.path.join(get_package_share_directory(package_description), 'config', amcl_config_real), '"'
    ])

    use_sim_time_value  = PythonExpression([
            '"', "True", '" if "', map_file_f, '" == "warehouse_map_keepout_sim.yaml" else "', "False", '"'
        ])

    cmd_vel_topic = PythonExpression([
            '"', "/diffbot_base_controller/cmd_vel_unstamped", '" if "',
             map_file_f, '" == "warehouse_map_keepout_sim.yaml" else "', "/cmd_vel", '"'
        ])

    # RVIZ Configuration
    rviz_config_dir = os.path.join(get_package_share_directory(package_description), 'rviz', 'amcl_rviz.rviz')


    map_server_node = Node(
                        package='nav2_map_server',
                        executable='map_server',
                        name='map_server',
                        output='screen',
                        parameters=[{'use_sim_time': use_sim_time_value},
                                    {'yaml_filename':map_file}
                                ]
                    )

    amcl_node = Node(
                    package='nav2_amcl',
                    executable='amcl',
                    name='amcl',
                    output='screen',
                    parameters=[nav2_yaml]          
        )

    lifecycle_manager_node = Node(
                            package='nav2_lifecycle_manager',
                            executable='lifecycle_manager',
                            name='lifecycle_manager_mapper',
                            output='screen',
                            parameters=[{'use_sim_time': use_sim_time_value},
                                        {'autostart': True},
                                        {'node_names': ['map_server','amcl']}]) 

    rviz_node = Node(
                package='rviz2',
                executable='rviz2',
                output='screen',
                name='rviz_node',
                parameters=[{'use_sim_time': use_sim_time_value}],
                arguments=['-d', rviz_config_dir])

    # Start the service server to make the robot go under the shelf and attach it 
    service_server_pkg = get_package_share_directory('approach_cart_service_server')
    
    approach_cart_service_server_node = IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(service_server_pkg, 'launch', 'start_approach_service_server.launch.py'),
                        ),
                        launch_arguments={'cmd_vel_topic_name': cmd_vel_topic}.items() 
                    )



    return LaunchDescription([
        map_file_arg,
        approach_cart_service_server_node,
        # rviz_node,
        lifecycle_manager_node,
        map_server_node,
        amcl_node,       
        ])
