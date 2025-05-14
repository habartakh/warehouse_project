import os
import launch

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time_arg =  launch.actions.DeclareLaunchArgument("use_sim_time", default_value="True")
    use_sim_time_f = LaunchConfiguration('use_sim_time')

    # Config file for the simulation
    if use_sim_time_f == True:
        configuration_basename = 'cartographer_sim.lua'
    #For the real robot 
    else:
        configuration_basename = 'cartographer_real.lua'
    
    
    
    cartographer_config_dir = os.path.join(get_package_share_directory('cartographer_slam'), 'config')
    

    rviz_config_dir = os.path.join(get_package_share_directory('cartographer_slam'), 'rviz', 'mapping.rviz')


    cartographer_node = Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', configuration_basename])


    cartographer_occupancy_grid_node = Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            output='screen',
            name='occupancy_grid_node',
            parameters=[{'use_sim_time': True}],
            arguments=['-resolution', '0.05', '-publish_period_sec', '1.0'])

    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            name='rviz_node',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', rviz_config_dir]
    )


    return LaunchDescription([
        use_sim_time_arg,
        cartographer_node,
        cartographer_occupancy_grid_node,
        rviz_node,    
    ])
