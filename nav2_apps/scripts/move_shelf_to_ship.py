#! /usr/bin/env python3


import time
from copy import deepcopy

from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
import rclpy
from rclpy.node import Node


from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from approach_cart_service_server.srv import GoToLoading


class SimpleCommander(Node):
    def __init__(self) :
        super().__init__('simple_commander')
        self.navigator = BasicNavigator()
        self.initial_pose = PoseStamped()
        self.loading_pose = PoseStamped()

        # client attributes to call the service /approach-shelf
        self.cli = self.create_client(GoToLoading, '/approach_shelf')
        self.req = GoToLoading.Request()

        
    def control_loop(self):
        self.set_initial_pose()
        
        # Wait for navigation to activate fully
        self.navigator.waitUntilNav2Active()

        # Then gor to the loading position
        self.go_to_loading_position()

        # After placing the robot in the loading position,
        # Move it under the shelf and lift the shelf
        self.go_under_shelf()

    
    # Set the robot initial pose
    def set_initial_pose(self):
        self.initial_pose.header.frame_id = 'map'
        self.initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.initial_pose.pose.position.x = -0.0499 # from TF coordinates in RVIZ
        self.initial_pose.pose.position.y = 0.016999
        self.initial_pose.pose.orientation.z = 0.0
        self.initial_pose.pose.orientation.w = 1.0
        self.navigator.setInitialPose(self.initial_pose)

    def go_to_loading_position(self):
        
        # set the coordinates of the loading location
        self.loading_pose.header.frame_id = 'map'
        self.loading_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        # Coordinates obtained via RVIZ
        self.loading_pose.pose.position.x = 5.64287
        self.loading_pose.pose.position.y = 0.0548694
        self.loading_pose.pose.orientation.z = -0.706825181105366
        self.loading_pose.pose.orientation.w = 0.7073882691671998
        print('Received request to go to the loading position.')
        self.navigator.goToPose(self.loading_pose)

        # print the remaining time to complete the navigation
        i = 0
        while not self.navigator.isTaskComplete():
            i = i + 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:
                print('Estimated time of arrival at loading position: ' + '{0:.0f}'.format(
                        Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                    + ' seconds.')


        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Arrived to the loading position!')
            
        elif result == TaskResult.CANCELED:
            print('Task at  was canceled. Returning to starting point...')
            self.initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            self.navigator.goToPose(self.initial_pose)

        elif result == TaskResult.FAILED:
            print('Task failed!')
            exit(-1)

    # To go under the shelf and load it,
    #  we will call a service made in a previous project:
    # https://github.com/habartakh/checkpoint9/blob/main/attach_shelf/src/approach_service_server.cpp
    def go_under_shelf(self):
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self.req.attach_to_shelf = True  # Initiate the robot movement towards the shelf
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)

        result = future.result()
        if (result.complete):
            print("Successfully lifted the shelf!")
        else : 
            print("Could not load the shelf...")
            exit(-1)


    def go_to_shipping_position():
        pass

    
# Position(5.54661, -0.152618, 0), Orientation(0, 0, -0.693247, 0.7207) = Angle: -1.53197
def main():

    rclpy.init()

    simple_commander = SimpleCommander()
    simple_commander.control_loop()
    exit(0)


if __name__ == '__main__':
    main()
