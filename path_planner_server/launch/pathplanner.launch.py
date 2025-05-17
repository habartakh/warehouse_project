import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PythonExpression
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution



def generate_launch_description():

    package_description = "path_planner_server"

    use_sim_time_arg =  launch.actions.DeclareLaunchArgument("use_sim_time", default_value="True")
    use_sim_time_f = LaunchConfiguration('use_sim_time')

    # The config files corresponding to either the real or simulated robot are chosen depending on the parameter use_sim_time
    controller_yaml = PythonExpression([
        '"', os.path.join(get_package_share_directory(package_description), 'config', 'controller_sim.yaml'), 
        '" if "', use_sim_time_f, '" == "True" else "', 
        os.path.join(get_package_share_directory(package_description), 'config', 'controller_real.yaml'), '"'
    ])
    
    
    bt_navigator_yaml = PythonExpression([
        '"', os.path.join(get_package_share_directory(package_description), 'config', 'bt_navigator_sim.yaml'), 
        '" if "', use_sim_time_f, '" == "True" else "', 
        os.path.join(get_package_share_directory(package_description), 'config', 'bt_navigator_real.yaml'), '"'
    ])


    planner_yaml = PythonExpression([
        '"', os.path.join(get_package_share_directory(package_description), 'config', 'planner_sim.yaml'), 
        '" if "', use_sim_time_f, '" == "True" else "', 
        os.path.join(get_package_share_directory(package_description), 'config', 'planner_real.yaml'), '"'
    ])


    recovery_yaml = PythonExpression([
        '"', os.path.join(get_package_share_directory(package_description), 'config', 'recoveries_sim.yaml'), 
        '" if "', use_sim_time_f, '" == "True" else "', 
        os.path.join(get_package_share_directory(package_description), 'config', 'recoveries_real.yaml'), '"'
    ])
   
    sim_topic_name = '/diffbot_base_controller/cmd_vel_unstamped'
    real_topic_name = '/cmd_vel'
    cmd_vel_topic_name = PythonExpression([
            '"', sim_topic_name, '" if "', use_sim_time_f, '" == "True" else "', real_topic_name, '"'
        ])
    # RVIZ Configuration
    rviz_config_dir = os.path.join(get_package_share_directory(package_description), 'rviz', 'pathplanning.rviz')

    rviz2 = Node(
                package='rviz2',
                executable='rviz2',
                output='screen',
                name='rviz_node',
                parameters=[{'use_sim_time': use_sim_time_f}],
                arguments=['-d', rviz_config_dir])


    planner_server = Node(
                    package='nav2_planner',
                    executable='planner_server',
                    name='planner_server',
                    remappings=[
                        ('/cmd_vel', cmd_vel_topic_name),
                    ],
                    output='screen',
                    parameters=[planner_yaml])
            
    controller_server = Node(
                        package='nav2_controller',
                        executable='controller_server',
                        name='controller_server',
                        remappings=[
                        ('/cmd_vel', cmd_vel_topic_name),
                        ],
                        output='screen',
                        parameters=[controller_yaml])
    
    recoveries_server =  Node(
                        package='nav2_behaviors',
                        executable='behavior_server',
                        name='recoveries_server',
                        remappings=[
                        ('/cmd_vel', cmd_vel_topic_name),
                        ],
                        parameters=[recovery_yaml],
                        output='screen')
    
    bt_navigator = Node(
                        package='nav2_bt_navigator',
                        executable='bt_navigator',
                        name='bt_navigator',
                        remappings=[
                        ('/cmd_vel', cmd_vel_topic_name),
                        ],
                        output='screen',
                        parameters=[bt_navigator_yaml])
    
    lifecycle_manager_pathplanner = Node(
                                        package='nav2_lifecycle_manager',
                                        executable='lifecycle_manager',
                                        name='lifecycle_manager',
                                        output='screen',
                                        parameters=[{'autostart': True},
                                                    {'node_names': [
                                                                    'controller_server',
                                                                    'planner_server',
                                                                    'recoveries_server',
                                                                    'bt_navigator']}])
   
    return LaunchDescription([ 
        use_sim_time_arg,
        rviz2,   
        lifecycle_manager_pathplanner,
        planner_server,
        controller_server, 
        recoveries_server,
        bt_navigator,
    ])
