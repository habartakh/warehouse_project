#include "approach_cart_service_server/srv/go_to_loading.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2/transform_datatypes.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <numeric>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using GoToLoading = approach_cart_service_server::srv::GoToLoading;

class ApproachShelfServer : public rclcpp::Node {
public:
  ApproachShelfServer() : Node("service_stop") {

    this->declare_parameter("cmd_vel_topic_name",
                            "/diffbot_base_controller/cmd_vel_unstamped");
    cmd_vel_topic_name = this->get_parameter("cmd_vel_topic_name")
                             .get_parameter_value()
                             .get<std::string>();

    std::cout << "cmd_vel_topic_name : " << cmd_vel_topic_name << std::endl;

    srv_cbg =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    srv_ = create_service<GoToLoading>(
        "approach_shelf",
        std::bind(&ApproachShelfServer::service_callback, this, _1, _2),
        rmw_qos_profile_services_default, srv_cbg);

    scan_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions options1;
    options1.callback_group = scan_callback_group_;

    scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&ApproachShelfServer::scan_callback, this, _1),
        options1);

    // Adjust the shelf legs intensity threshold and the odom frame name based
    // on whether the robot is in the simulated or real environment
    if (cmd_vel_topic_name == "/cmd_vel") {
      // real robot
      std::cout << "Service server for real robot !! " << std::endl;
      scan_intensity_thres = 4500;
      odom_frame_name = "robot_odom";
      distance_under_shelf = 0.3;
    } else {
      std::cout << "Service server for SIM robot !! " << std::endl;
      scan_intensity_thres = 8000;
      odom_frame_name = "odom";
      distance_under_shelf = 0.45;
    }

    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        cmd_vel_topic_name, 10);

    shelf_detection_cp = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    move_cart_cp = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    shelf_detection_timer = this->create_wall_timer(
        500ms, std::bind(&ApproachShelfServer::shelf_leg_detection, this),
        shelf_detection_cp);
    shelf_detection_timer->cancel(); // Cancel the timer till service start

    move_cart_center_timer = this->create_wall_timer(
        500ms, std::bind(&ApproachShelfServer::move_cart_legs_center, this),
        move_cart_cp);
    move_cart_center_timer->cancel(); // Cancel the timer till service start

    tf_static_broadcaster_ =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // change the durability to the same one as RVIZ for compatibility
    rclcpp::QoS qos_profile(rclcpp::KeepLast(1));
    qos_profile.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    lift_shelf_pub = this->create_publisher<std_msgs::msg::String>(
        "elevator_up", qos_profile);
  }

private:
  rclcpp::CallbackGroup::SharedPtr srv_cbg;
  rclcpp::Service<GoToLoading>::SharedPtr srv_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr lift_shelf_pub;

  rclcpp::CallbackGroup::SharedPtr shelf_detection_cp;
  rclcpp::CallbackGroup::SharedPtr move_cart_cp;
  rclcpp::TimerBase::SharedPtr shelf_detection_timer;
  rclcpp::TimerBase::SharedPtr move_cart_center_timer;

  rclcpp::CallbackGroup::SharedPtr scan_callback_group_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
  std::string odom_frame_name;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  std::vector<int> shelf_laser_indexes;
  int scan_intensity_thres;
  int leg_2_first_index;
  sensor_msgs::msg::LaserScan::SharedPtr laser_scan_msg;
  float angle_increment;
  bool two_legs_detected = false;
  bool published_cart_frame = false;
  bool service_complete = false;
  bool reached_final_position = false;
  bool leg_detection_complete = false;
  float distance_under_shelf;

  std::string cmd_vel_topic_name;

  void service_callback(const std::shared_ptr<GoToLoading::Request> request,
                        const std::shared_ptr<GoToLoading::Response> response) {
    rclcpp::Rate loop_rate(10);
    service_complete = false;
    shelf_detection_timer->reset();

    // wait till the robot received valid scan data
    while (rclcpp::ok() && !leg_detection_complete) {
      loop_rate.sleep();
    }

    if (two_legs_detected) {

      shelf_detection_timer->cancel();

      // Publish the cart_frame
      publish_cart_frame_tf();
      RCLCPP_INFO(this->get_logger(), " Published cart_frame! ");

      // if the service request is true, move the robot to the shelf and lift it
      if (request->attach_to_shelf) {

        reached_final_position = false;

        move_cart_center_timer->reset();

        // wait till the robot is under the cart to return service response

        while (rclcpp::ok() && !service_complete) {
          loop_rate.sleep();
        }

        move_cart_center_timer->cancel();
        response->complete = true;

      } else {
        RCLCPP_INFO_ONCE(this->get_logger(),
                         " The final approach was not requested.");
        response->complete = false;
      }

    } else {
      // if only one or no legs were detected, return false
      RCLCPP_INFO_ONCE(this->get_logger(), "Unable to detect two shelf legs.");
      response->complete = false;
    }
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

    if (msg != nullptr) {
      angle_increment = msg->angle_increment;
      laser_scan_msg = msg;
    } else {
      RCLCPP_INFO(this->get_logger(), " No laser_scan message received yet...");
    }
  }

  void shelf_leg_detection() {
    if (laser_scan_msg != nullptr) {
      // store all the indexes of the rays having intensities >=
      // scan_intensity_thres that correspond to the shelf legs inside a vector
      for (auto it = laser_scan_msg->intensities.begin();
           it != laser_scan_msg->intensities.end(); ++it) {
        if (*it >= scan_intensity_thres) {
          int shelf_index = it - laser_scan_msg->intensities.begin();
          shelf_laser_indexes.push_back(shelf_index);
        }
      }

      // find the number of the shelf legs detected
      // if two legs were detected, the intensity indexes won't be consecutive
      auto it = std::adjacent_find(shelf_laser_indexes.begin(),
                                   shelf_laser_indexes.end(),
                                   [](int x, int y) { return y != x + 1; });
      leg_2_first_index = it - shelf_laser_indexes.begin();

      // if the two legs were detected
      if (it != shelf_laser_indexes.end()) {
        two_legs_detected = true;
      }
      leg_detection_complete = true;
    }
  }

  // Publish a transform named cart_frame to the center point between both legs
  void publish_cart_frame_tf() {

    // Compute the coordinates of the point in the middle of the 2 legs
    // First, compute projection of both legs on x & y axes
    int middle_ray_index = (int)laser_scan_msg->intensities.size() / 2;

    int leg_1_index = shelf_laser_indexes[0];
    double leg1_angle = (leg_1_index - middle_ray_index) * angle_increment;
    double leg1_x = laser_scan_msg->ranges[leg_1_index] * std::cos(leg1_angle);
    double leg1_y = laser_scan_msg->ranges[leg_1_index] * std::sin(leg1_angle);

    int leg_2_index = shelf_laser_indexes.back();
    double leg2_angle = (leg_2_index - middle_ray_index) * angle_increment;
    double leg2_x = laser_scan_msg->ranges[leg_2_index] * std::cos(leg2_angle);
    double leg2_y = laser_scan_msg->ranges[leg_2_index] * std::sin(leg2_angle);

    double middle_point_x = (leg1_x + leg2_x) / 2.0;
    double middle_point_y = (leg1_y + leg2_y) / 2.0;
    double middle_point_distance = std::sqrt(middle_point_x * middle_point_x +
                                             middle_point_y * middle_point_y);
    double middle_point_angle = std::atan2(middle_point_y, middle_point_x);

    // To get the tf between odom and the cart frame :
    // First, compute the pose of the cart_frame origin relative to laser_frame
    // Then, get the coordinates of the cart_frame origin relative to odom_frame
    geometry_msgs::msg::PoseStamped laser_pose;
    laser_pose.header.frame_id = "robot_front_laser_base_link";
    laser_pose.header.stamp = this->get_clock()->now();
    laser_pose.pose.position.x =
        middle_point_distance * std::cos(middle_point_angle);
    laser_pose.pose.position.y =
        middle_point_distance * std::sin(middle_point_angle);
    laser_pose.pose.position.z = 0.0;
    laser_pose.pose.orientation.w = 1.0;

    geometry_msgs::msg::PoseStamped odom_pose;
    geometry_msgs::msg::TransformStamped odom_to_laser_tf;
    try {
      odom_to_laser_tf = tf_buffer_->lookupTransform(
          odom_frame_name, "robot_front_laser_base_link", tf2::TimePointZero);
      tf2::doTransform(laser_pose, odom_pose, odom_to_laser_tf);
    }

    catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
      return;
    }

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = odom_frame_name;
    t.child_frame_id = "cart_frame";
    t.transform.translation.x = odom_pose.pose.position.x;
    t.transform.translation.y = odom_pose.pose.position.y;
    t.transform.translation.z = 0.0;

    // Orientation
    tf2::Quaternion q;
    q.setRPY(0, 0, -1.57);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    // Send the transformation
    tf_static_broadcaster_->sendTransform(t);
  }

  void move_cart_legs_center() {
    std::string fromFrameRel = "cart_frame";
    std::string toFrameRel = "robot_base_footprint";
    geometry_msgs::msg::TransformStamped t;

    // Look up for the transformation between target_frame and robot frames
    // and send velocity commands for robot to reach target_frame
    if (!reached_final_position) {

      try {
        t = tf_buffer_->lookupTransform(toFrameRel, fromFrameRel,
                                        tf2::TimePointZero);
      } catch (const tf2::TransformException &ex) {
        RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s",
                    toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
        return;
      }

      auto x = t.transform.translation.x;
      auto y = t.transform.translation.y;

      float error_distance = std::sqrt(x * x + y * y);
      float error_yaw = std::atan2(y, x);
      //   RCLCPP_INFO(this->get_logger(), "Error distance: %.4f",
      //   error_distance); RCLCPP_INFO(this->get_logger(), "Error yaw: %.4f",
      //   error_yaw);

      geometry_msgs::msg::Twist msg;

      if (std::abs(error_distance) > 0.05) {
        msg.angular.z = 1.0 * error_yaw;
        msg.linear.x = 0.1;
        publisher_->publish(msg);

      } else {

        RCLCPP_INFO(this->get_logger(),
                    "Successfully approached cart legs center.");
        move_under_cart();
        RCLCPP_INFO(this->get_logger(), "Successfully moved under the cart.");

        reached_final_position = true;
        service_complete = true;
      }
    }
  }

  // Move 45 cm along the x axis to go under the cart
  void move_under_cart() {
    RCLCPP_INFO(this->get_logger(),
                "Proceeding to move the robot under the cart.");

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.15;

    std::string odom_frame = odom_frame_name;
    std::string base_frame = "robot_base_footprint";

    geometry_msgs::msg::TransformStamped start_tf;
    try {
      start_tf = tf_buffer_->lookupTransform(odom_frame_name, base_frame,
                                             tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "TF error: %s", ex.what());
      return;
    }

    double start_x = start_tf.transform.translation.x;
    double start_y = start_tf.transform.translation.y;

    rclcpp::Rate rate(10); // 10 Hz

    while (rclcpp::ok()) {
      publisher_->publish(cmd);
      rate.sleep();

      geometry_msgs::msg::TransformStamped current_tf;
      try {
        current_tf = tf_buffer_->lookupTransform(odom_frame_name, base_frame,
                                                 tf2::TimePointZero);
      } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "TF error during move: %s", ex.what());
        continue;
      }

      // Compute the distance travelled
      double dx = current_tf.transform.translation.x - start_x;
      double dy = current_tf.transform.translation.y - start_y;
      double dist_moved = std::sqrt(dx * dx + dy * dy);

      if (dist_moved >= distance_under_shelf) {
        break;
      }
    }

    // Stop the robot
    cmd.linear.x = 0.0;
    publisher_->publish(cmd);

    lift_shelf();
    RCLCPP_INFO(this->get_logger(), "Successfully lifted the shelf.");
  }

  void lift_shelf() {
    std_msgs::msg::String lift_msg;
    lift_msg.data = "";
    lift_shelf_pub->publish(lift_msg);
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  std::shared_ptr<ApproachShelfServer> approach_server_node =
      std::make_shared<ApproachShelfServer>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(approach_server_node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}