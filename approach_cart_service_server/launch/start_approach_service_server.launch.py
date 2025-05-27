from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    
    cmd_vel_topic_name_arg = DeclareLaunchArgument(
        "cmd_vel_topic_name", default_value="/diffbot_base_controller/cmd_vel_unstamped")

    cmd_vel_topic_name_f = LaunchConfiguration('cmd_vel_topic_name')

    
    approach_service_node = Node(
            package='approach_cart_service_server',
            executable='approach_service_server_node',
            output='screen',
            name='approach_service_server_node',
            parameters=[{'cmd_vel_topic_name': cmd_vel_topic_name_f}],
            )

    return LaunchDescription([
        cmd_vel_topic_name_arg,
        approach_service_node,
    ])