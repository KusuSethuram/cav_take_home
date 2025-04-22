#pragma once


// Here we include message types which we can subscribe to or publish
#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <raptor_dbw_msgs/msg/wheel_speed_report.hpp>
#include <raptor_dbw_msgs/msg/steering_extended_report.hpp>


#include <raptor_dbw_msgs/msg/steering_extended_report.hpp>
#include <novatel_oem7_msgs/msg/rawimu.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>






class TakeHome : public rclcpp::Node {


 public:
  TakeHome(const rclcpp::NodeOptions& options);
  float position_x; float position_y; float position_z;
  float linearVx; float linearVy; float linearVz;
  float rrWheelSpeed; float rlWheelSpeed; float frWheelSpeed; float flWheelSpeed;
  float primary_steering_angle; float wheelAngle; float curvillinearDistance;
  float wf = 1.638; float wr = 1.523; float lf = 1.7238; float yaw; rclcpp::Time msg_time;
  rclcpp::Time current_time;
  void odometry_callback(nav_msgs::msg::Odometry::ConstSharedPtr odom_msg);
  void wheelSpeed_callback(raptor_dbw_msgs::msg::WheelSpeedReport::ConstSharedPtr wheel_msg);
  void steering_callback(raptor_dbw_msgs::msg::SteeringExtendedReport::ConstSharedPtr steering_msg);
  void slip_ratio_calcs();
  std::vector<rclcpp::Time> imu_timestamps_;


  rclcpp::Time last_lap_time_;
  std::vector<float> curvillinearDistances;
 


  std::vector<double> deltaTimes;
  void imu_callback(novatel_oem7_msgs::msg::RAWIMU::ConstSharedPtr imu_msg);
  void curvillinear_callback(const std_msgs::msg::Float32::SharedPtr curvillinear_msg);




  private:
  // Subscribers and Publishers
  // subscribers : one for odometry, wheelspeed report, and one for steering wheel angle
  rclcpp::Subscription<raptor_dbw_msgs::msg::WheelSpeedReport>::SharedPtr wheelSpeed_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
  rclcpp::Subscription<raptor_dbw_msgs::msg::SteeringExtendedReport>::SharedPtr steering_subscriber_;
  rclcpp::Subscription<novatel_oem7_msgs::msg::RAWIMU>::SharedPtr imu_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr curvillinear_subscriber_;




  //publishers for each wheel's slip ratio + keeping the example one
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr metric_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr slip_rr_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr slip_fr_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr slip_rl_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr slip_fl_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr jitter_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr lapTime_publisher_;


};



