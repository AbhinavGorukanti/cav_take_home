#pragma once

// Here we include message types which we can subscribe to or publish
#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/odometry.hpp> 

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>

#include <raptor_dbw_msgs/msg/wheel_speed_report.hpp>
#include <raptor_dbw_msgs/msg/steering_extended_report.hpp>

#include <novatel_oem7_msgs/msg/rawimu.hpp>

class TakeHome : public rclcpp::Node {
 public:
  TakeHome(const rclcpp::NodeOptions& options);

  void odometry_callback(nav_msgs::msg::Odometry::ConstSharedPtr odom_msg);
  
  void wheel_speed_callback(raptor_dbw_msgs::msg::WheelSpeedReport::ConstSharedPtr wspeed_msg);

  void steering_callback(raptor_dbw_msgs::msg::SteeringExtendedReport::ConstSharedPtr steering_msg);

  void imu_callback(novatel_oem7_msgs::msg::RAWIMU::ConstSharedPtr imu_msg);

  void lap_time_callback(std_msgs::msg::Float32::ConstSharedPtr dist_msg);

 private:

  // Subscribers and Publishers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
  rclcpp::Subscription<raptor_dbw_msgs::msg::WheelSpeedReport>::SharedPtr wheel_speed_subscriber_;
  rclcpp::Subscription<raptor_dbw_msgs::msg::SteeringExtendedReport>::SharedPtr steering_angle_subscriber_;
  rclcpp::Subscription<novatel_oem7_msgs::msg::RAWIMU>::SharedPtr imu_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr lap_time_subscriber_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr metric_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr rr_slip_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr rl_slip_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr fl_slip_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr fr_slip_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr imu_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr lap_time_publisher_;
  //Declare constant values and variables 
  const float wf = 1.638;
  const float wr = 1.523;
  const float lf = 1.7238;
  float vw_rr, vw_rl, vw_fr,vw_fl, steering_angle;
  std::vector<double> times;
  double lap_start = 0.0;
  bool first_lap = true;
};
