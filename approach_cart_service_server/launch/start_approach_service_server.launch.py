from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():
     
    approach_service_node = Node(
            package='approach_cart_service_server',
            executable='approach_service_server_node',
            output='screen',
            name='approach_service_server_node',
            )

    return LaunchDescription([
        approach_service_node,
    ])