#! /usr/bin/env python3


import time
from copy import deepcopy
from enum import Enum

from geometry_msgs.msg import PoseStamped, Polygon, Point32, Twist
from std_msgs.msg import String
from rclpy.duration import Duration
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from approach_cart_service_server.srv import GoToLoading

class SimpleCommander(Node):
    def __init__(self) :
        super().__init__('simple_commander')

        # publisher to unload the shelf with a compatible QoS profile
        my_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.unload_publisher = self.create_publisher(String, '/elevator_down', qos_profile=my_qos_profile)
        self.load_publisher = self.create_publisher(String, '/elevator_up', qos_profile=my_qos_profile)


    def unload_shelf(self):
        unload_msg = String()
        unload_msg.data = ""
        for i in range(10):
            self.unload_publisher.publish(unload_msg)
            print("Publishing to /elevator_down for real robot")
            time.sleep(2.0)
        self.get_logger().info('Unloaded the shelf')

    def load_shelf(self):
        load_msg = String()
        load_msg.data = ""
        for i in range(10):
            self.load_publisher.publish(load_msg)
            print("Publishing to /elevator_up for real robot")
            time.sleep(2.0)
        self.get_logger().info('loaded the shelf')


def main():
    rclpy.init()

    simple_commander = SimpleCommander()

    # simple_commander.load_shelf()
    
    simple_commander.unload_shelf()

    rclpy.shutdown()

if __name__ == '__main__':
    main()

