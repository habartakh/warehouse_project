#! /usr/bin/env python3


import time
from copy import deepcopy
from enum import Enum

from geometry_msgs.msg import PoseStamped, Polygon, Point32
from rclpy.duration import Duration
import rclpy
from rclpy.node import Node


from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from approach_cart_service_server.srv import GoToLoading

class State(Enum):
    UNINITIALIZED = 1 
    GO_LOADING = 2 
    GO_ATTACH_SHELF = 3
    GO_SHIPPING = 4
    RETURN_INIT_POS = 5



class SimpleCommander(Node):
    def __init__(self) :
        super().__init__('simple_commander')
        self.navigator = BasicNavigator()
        self.initial_pose = PoseStamped()
        self.loading_pose = PoseStamped()
        self.shipping_pose = PoseStamped()

        # client attributes to call the service /approach-shelf
        self.cli = self.create_client(GoToLoading, '/approach_shelf')
        self.req = GoToLoading.Request()
        self.sent_request = False 

        # robot footprint
        self.robot_footprint = Polygon()
        self.publisher_local = self.create_publisher(Polygon, '/local_costmap/footprint', 10)
        self.publisher_global = self.create_publisher(Polygon, '/global_costmap/footprint', 10)
        self.timer = self.create_timer(1.0, self.update_robot_footprint)
        # self.timer.cancel()
        self.update_is_true = False

        # Robot Actual state during the shiping position 
        self.robot_state = State.UNINITIALIZED

        # Timer for the control loop 
        self.control_timer = self.create_timer(0.5, self.control_loop)

        # Velocity commands publisher


        
    def control_loop(self):
        
        match self.robot_state : 
            case State.UNINITIALIZED:
                self.set_initial_pose()
            
            case State.GO_LOADING :    
                # Wait for navigation to activate fully
                self.navigator.waitUntilNav2Active()

                # Then go to the loading position
                self.go_to_loading_position()

            case State.GO_ATTACH_SHELF:
                # only send ONE request 
                if self.sent_request == False :  
                    # After placing the robot in the loading position,
                    # Move it under the shelf and lift the shelf
                    self.go_under_shelf()
                
            case State.GO_SHIPPING : 
                print('Inside State GO_SHIPPING')
                # Then update the robot footprint since it is carrying the shelf now
                # self.timer.reset()
                
                #self.go_to_shipping_position()
                

            case _ : 
                pass


    
    # Set the robot initial pose
    def set_initial_pose(self):
        self.initial_pose.header.frame_id = 'map'
        self.initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.initial_pose.pose.position.x = -0.0499 # from TF coordinates in RVIZ
        self.initial_pose.pose.position.y = 0.016999
        self.initial_pose.pose.orientation.z = 0.0
        self.initial_pose.pose.orientation.w = 1.0
        self.navigator.setInitialPose(self.initial_pose)

        self.robot_state = State.GO_LOADING

    # z = -0.7660446835233129 w = 0.6427873231837007 -> 100 degrees
    # z = -0.706825181105366 w = 0.7073882691671998 -> 90 degrees
    def go_to_loading_position(self):
        
        # set the coordinates of the loading location
        self.loading_pose.header.frame_id = 'map'
        self.loading_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        # Coordinates obtained via RVIZ
        self.loading_pose.pose.position.x = 5.64287
        self.loading_pose.pose.position.y = 0.0548694
        self.loading_pose.pose.orientation.z = -0.7660446835233129
        self.loading_pose.pose.orientation.w = 0.6427873231837007
        self.get_logger().info('Received request to go to the loading position.')
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
            self.get_logger().info('Arrived to the loading position!')
            self.robot_state = State.GO_ATTACH_SHELF
            
            
        elif result == TaskResult.CANCELED:
            self.get_logger().info('Task at  was canceled. Returning to starting point...')
            self.initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            self.navigator.goToPose(self.initial_pose)

        elif result == TaskResult.FAILED:
            self.get_logger().info('Task failed!')
            exit(-1)

    # To go under the shelf and load it,
    #  we will call a service made in a previous project:
    # https://github.com/habartakh/checkpoint9/blob/main/attach_shelf/src/approach_service_server.cpp
    def go_under_shelf(self):
        print('Inside go_under_shelf function')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self.req.attach_to_shelf = True  # Initiate the robot movement towards the shelf
        self.future = self.cli.call_async(self.req)
        self.future.add_done_callback(self.service_response_callback)

        self.sent_request = True 
        
        

    def service_response_callback(self,future):
        
        result = future.result()
        if (result.complete):
            self.get_logger().info("Successfully lifted the shelf!")
            self.update_is_true = True  # Update the robot_footprint
            self.robot_state = State.GO_SHIPPING # update the navigation state
        else : 
            self.get_logger().info("Could not load the shelf...")
            exit(-1)

    # if update is true, then the robot is carrying the shelf
    # we update the footprint to a bigger polygon and 
    # publish it to the local & global costmaps
    def update_robot_footprint(self):
        robot_only_ft = [
            Point32(x=0.3, y=0.3, z=0.00),
            Point32(x=0.3, y=-0.3, z=0.00),
            Point32(x=-0.3, y=-0.3, z=0.00),
            Point32(x=-0.3, y=0.3, z=0.00)]
        
        robot_shelf_footprint = [
            Point32(x=0.5, y=0.5, z=0.00),
            Point32(x=0.5, y=-0.5, z=0.00),
            Point32(x=-0.5, y=-0.5, z=0.00),
            Point32(x=-0.5, y=0.5, z=0.00)]

        self.robot_footprint.points = robot_shelf_footprint if self.update_is_true else robot_only_ft
        self.publisher_local.publish(self.robot_footprint)
        self.publisher_global.publish(self.robot_footprint)

        self.get_logger().info("Updated the robot footprint")


    
    def go_to_shipping_position(self):
        '''
        shipping pose : Frame:map, Position(2.51228, 1.1686, 0), 
        Orientation(0, 0, 0.701893, 0.712283) = Angle: 1.5561
        '''
        # set the coordinates of the shipping location
        self.shipping_pose.header.frame_id = 'map'
        self.shipping_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        # Coordinates obtained via RVIZ
        self.shipping_pose.pose.position.x = 2.51228
        self.shipping_pose.pose.position.y = 1.1686
        self.shipping_pose.pose.orientation.z = 0.701893
        self.shipping_pose.pose.orientation.w = 0.712283
        self.get_logger().info('Received request to go to the shipping position.')
        self.navigator.goToPose(self.shipping_pose)

        # print the remaining time to complete the navigation
        i = 0
        while not self.navigator.isTaskComplete():
            i = i + 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:
                print('Estimated time of arrival at shipping position: ' + '{0:.0f}'.format(
                        Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                    + ' seconds.')


        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Arrived to the shipping position!')
            
        elif result == TaskResult.CANCELED:
            self.get_logger().info('Task at  was canceled. Returning to starting point...')
            self.initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            self.navigator.goToPose(self.initial_pose)

        elif result == TaskResult.FAILED:
            self.get_logger().info('Task failed!')
            exit(-1)

    
def main():

    rclpy.init()

    simple_commander = SimpleCommander()
    rclpy.spin(simple_commander)

    rclpy.shutdown()
    
    # exit(0)


if __name__ == '__main__':
    main()
