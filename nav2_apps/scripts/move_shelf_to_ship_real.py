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

        # Timer for the control loop
        self.control_ran_once = False
        self.reentrant_group = ReentrantCallbackGroup() 
        self.control_timer = self.create_timer(2.0, self.control_loop,callback_group=self.reentrant_group)

        # client attributes to call the service /approach-shelf
        self.service_callback_group = ReentrantCallbackGroup()
        self.cli = self.create_client(GoToLoading, '/approach_shelf')
        self.req = GoToLoading.Request()
        self.sent_request = False 

        # robot footprint
        self.mutually_exclusive_group_2 = MutuallyExclusiveCallbackGroup() 
        self.robot_footprint = Polygon()
        self.publisher_local = self.create_publisher(Polygon, '/local_costmap/footprint', 10)
        self.publisher_global = self.create_publisher(Polygon, '/global_costmap/footprint', 10)
        self.update_footprint_timer = self.create_timer(1.0, self.update_robot_footprint, callback_group= self.mutually_exclusive_group_2)
        self.update_is_true = False

        # Robot Actual state during the shiping position 
        self.robot_state = State.UNINITIALIZED

        # Velocity commands publisher
        self.vel_cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.reverse = False

        # publisher to unload the shelf with a compatible QoS profile
        my_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.unload_publisher = self.create_publisher(String, '/elevator_down', qos_profile=my_qos_profile)

        # Wait for navigation to activate fully
        self.navigator.waitUntilNav2Active()

        
    def control_loop(self):
        self.control_timer.cancel()
        
        match self.robot_state : 
            case State.UNINITIALIZED:
                self.set_initial_pose()
            
            case State.GO_LOADING :    
                # Go to the loading position
                self.go_to_loading_position()

            case State.GO_ATTACH_SHELF:
                # only send ONE request 
                #if self.sent_request == False :  
                    # After placing the robot in the loading position,
                    # Move it under the shelf and lift the shelf,
                    # Then update the robot footprint
                self.go_under_shelf()
                
            case State.GO_SHIPPING : 
                self.update_is_true = True

                if not self.reverse : 
                    # First, take a step back from the obstacles 
                    self.back_up(80)
                
                #Then go to the shipping position
                self.go_to_shipping_position()
                
            case State.RETURN_INIT_POS:
                 
                # Go back to your initial position
                self.navigator.goToPose(self.initial_pose)
                while not self.navigator.isTaskComplete():
                    pass
                
                result = self.navigator.getResult()
                if result == TaskResult.SUCCEEDED:
                    self.get_logger().info("Mission accomplished!")
                else:
                    self.get_logger().info("Could not return to the initial position...")

            case _ : 
               print ("Default case") 


    
    # Set the robot initial pose
    '''
    Frame:map, Position(-0.803685, -0.0706855, 0), 
    Orientation(0,0, -0.0872494, 0.996187) = Angle: -0.174721
    '''
    def set_initial_pose(self):
        self.initial_pose.header.frame_id = 'map'
        self.initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.initial_pose.pose.position.x = -0.803685 # from TF coordinates in RVIZ
        self.initial_pose.pose.position.y = -0.0706855
        self.initial_pose.pose.orientation.z = 0.0
        self.initial_pose.pose.orientation.w = 1.0
        self.navigator.setInitialPose(self.initial_pose)

        self.robot_state = State.GO_LOADING
        self.control_timer.reset()

    # z = -0.7660446835233129 w = 0.6427873231837007 -> 100 degrees
    # z = -0.706825181105366 w = 0.7073882691671998 -> 90 degrees
    def go_to_loading_position(self):
        '''
        Frame:map, Position(3.43772, -1.16119, 0), 
        Orientation(0, 0, -0.8312207978286242, 0.5559424298046924) = Angle: -1.9626
        '''
        # set the coordinates of the loading location
        self.loading_pose.header.frame_id = 'map'
        self.loading_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        # Coordinates obtained via RVIZ
        self.loading_pose.pose.position.x = 3.63772
        self.loading_pose.pose.position.y = -1.16119
        self.loading_pose.pose.orientation.z = -0.8312207978286242
        self.loading_pose.pose.orientation.w = 0.5559424298046924
        self.get_logger().info('Received request to go to the loading position.')
        self.navigator.goToPose(self.loading_pose)

        # print the remaining time to complete the navigation
        i = 0
        while not self.navigator.isTaskComplete():
            i = i + 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 15 == 0:
                print('Estimated time of arrival at loading position: ' + '{0:.0f}'.format(
                        Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                    + ' seconds.')


        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Arrived to the loading position!')
            self.robot_state = State.GO_ATTACH_SHELF
            self.control_timer.reset()
            
            
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
        self.sent_request = True 
        
        while(not self.future.done()):
            # print("self.future is not done yet!!")
            self.create_rate(0.5).sleep()

        result = self.future.result()
        if (result.complete):
            self.get_logger().info("Successfully lifted the shelf!")
            self.update_is_true = True  # Update the robot_footprint
            self.robot_state = State.GO_SHIPPING # update the navigation state
            self.control_timer.reset()
            
        else : 
            self.get_logger().info("Could not load the shelf...")
            exit(-1)

        
        
    # The function called when the service is done 
    def service_response_callback(self):
        result = self.future.result()
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
            Point32(x=0.45, y=0.45, z=0.00),
            Point32(x=0.45, y=-0.45, z=0.00),
            Point32(x=-0.45, y=-0.45, z=0.00),
            Point32(x=-0.45, y=0.45, z=0.00)]

        self.robot_footprint.points = robot_shelf_footprint if self.update_is_true else robot_only_ft
        self.publisher_local.publish(self.robot_footprint)
        self.publisher_global.publish(self.robot_footprint)

        #self.get_logger().info("Updated the robot footprint")

    # After lifting the shelf, back up the robot a little
    # sincs the new footprint is larger and overlaps with some of the warehouse detected obstacles
    # Thus, the navigation system won't be able to plan a suitable path to the shipping position
    def back_up(self, duration):
        cmd_vel_msg = Twist()
        cmd_vel_msg.angular.z = 0.0 
        cmd_vel_msg.linear.x = -0.2 

        self.get_logger().info('Moving backwards...')
        
        for _ in range(duration):
            self.vel_cmd_publisher.publish(cmd_vel_msg)
            time.sleep(0.1)  # Wait 2 seconds

        cmd_vel_msg.linear.x = 0.0       
        
        self.get_logger().info('Stopping...')
        self.vel_cmd_publisher.publish(cmd_vel_msg)
        time.sleep(1.0)

        self.reverse = True

    # For the real robot, the elevator does not respond to 
    # publishing to /elevator_down only once
    # Thus, we publish multiple times the same message 
    def unload_shelf(self):
        unload_msg = String()
        unload_msg.data = ""
        for i in range(10):
            self.unload_publisher.publish(unload_msg)
            print("Publishing to /elevator_down for real robot")
            time.sleep(2.0)
        self.get_logger().info('Unloaded the shelf')
    
    def go_to_shipping_position(self):
        '''
        shipping pose : 
        Frame:map, Position(1.15965, 0.697947, 0), 
        Orientation(0, 0, 0.576639, 0.816999) = Angle: 1.22922
        '''
        # set the coordinates of the shipping location
        self.shipping_pose.header.frame_id = 'map'
        self.shipping_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        # Coordinates obtained via RVIZ
        self.shipping_pose.pose.position.x = 1.15965
        self.shipping_pose.pose.position.y = 0.697947
        self.shipping_pose.pose.orientation.z = 0.576639
        self.shipping_pose.pose.orientation.w = 0.816999
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
            self.unload_shelf() # Unload the shelf
            self.update_is_true = False # Then reset the robot footprint
            self.back_up(80)
            self.robot_state = State.RETURN_INIT_POS
            
        elif result == TaskResult.CANCELED:
            self.get_logger().info('Shipping Task was canceled. Returning to starting point...')
            self.initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            self.navigator.goToPose(self.initial_pose)

        elif result == TaskResult.FAILED:
            self.get_logger().info('Task failed!')
            exit(-1)
        self.control_timer.reset()

    
def main():

    rclpy.init()

    simple_commander = SimpleCommander()
    
    # Use a MultiThreadedExecutor to enable parallel execution
    executor = MultiThreadedExecutor()
    executor.add_node(simple_commander)
    executor.spin()

    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
