#include "take_home_node/take_home.hpp" //own header file
#include <rclcpp_components/register_node_macro.hpp> //makes it a component?
#include <vector>  // For storing timestamps in a vector








TakeHome::TakeHome(const rclcpp::NodeOptions& options) //constructs the node
    : Node("take_home_metrics", options) {


    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(); //keeplast stores last 10 messages, best effort don't gaurantee delivery
   
    // Look at the hpp file to define all class variables, including subscribers
    // A subscriber will "listen" to a topic and whenever a message is published to it, the subscriber
    // will pass it onto the attached callback (`TakeHome::odometry_callback` in this instance)
      odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "vehicle/uva_odometry", qos_profile,
      std::bind(&TakeHome::odometry_callback, this, std::placeholders::_1));
   
      wheelSpeed_subscriber_ = this->create_subscription<raptor_dbw_msgs::msg::WheelSpeedReport>(
      "/raptor_dbw_interface/wheel_speed_report", qos_profile,
      std::bind(&TakeHome::wheelSpeed_callback, this, std::placeholders::_1));


      curvillinear_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
        "/curvilinear_distance", 10,
        std::bind(&TakeHome::curvillinear_callback, this, std::placeholders::_1));
     


      steering_subscriber_ = this->create_subscription<raptor_dbw_msgs::msg::SteeringExtendedReport>(
        "/raptor_dbw_interface/steering_extended_report", qos_profile,
        std::bind(&TakeHome::steering_callback, this, std::placeholders::_1)
      );


      imu_subscriber_ = this->create_subscription<novatel_oem7_msgs::msg::RAWIMU>(
        "/novatel_top/rawimu", rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
        std::bind(&TakeHome::imu_callback, this, std::placeholders::_1)
    );
   
      metric_publisher_ = this->create_publisher<std_msgs::msg::Float32>("metrics_output", qos_profile);
      slip_rr_publisher_ = this->create_publisher<std_msgs::msg::Float32>("slip/long/rr", qos_profile);
      slip_rl_publisher_ = this->create_publisher<std_msgs::msg::Float32>("slip/long/rl", qos_profile);
      slip_fl_publisher_ = this->create_publisher<std_msgs::msg::Float32>("slip/long/fl", qos_profile);
      slip_fr_publisher_ = this->create_publisher<std_msgs::msg::Float32>("slip/long/fr", qos_profile);
      jitter_publisher_ = this->create_publisher<std_msgs::msg::Float32>("imu_top/jitter", qos_profile);
      lapTime_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/lap_time", qos_profile);


      last_lap_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());


}


//
/**
 * Whenever a message is published to the topic "vehicle/uva_odometry" the subscriber passes the message onto this callback
 * To see what is in each message look for the corresponding .msg file in this repository
 * For instance, when running `ros2 bag info` on the given bag, we see the wheel speed report has message type of raptor_dbw_msgs/msgs/WheelSpeedReport
 * and from the corresponding WheelSpeedReport.msg file we see that we can do msg->front_left for the front left speed for instance.
 * For the built in ROS2 messages, we can find the documentation online: e.g. https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html
 */
void TakeHome::odometry_callback(nav_msgs::msg::Odometry::ConstSharedPtr odom_msg) {
   position_x = odom_msg->pose.pose.position.x;
   position_y = odom_msg->pose.pose.position.y;
   position_z = odom_msg->pose.pose.position.z;
  //get odom data for velocities
   linearVx = odom_msg->twist.twist.linear.x;
   linearVy = odom_msg->twist.twist.linear.y;
   linearVz = odom_msg->twist.twist.linear.z;


  //yaw rate
   yaw = odom_msg->twist.twist.angular.z;


  // Do stuff with this callback! or more, idc
  std_msgs::msg::Float32 metric_msg;
  metric_msg.data = (position_x + position_y + position_z) / (position_x + position_z); // Example metric calculation
  metric_publisher_->publish(metric_msg);
}


void TakeHome::wheelSpeed_callback(raptor_dbw_msgs::msg::WheelSpeedReport::ConstSharedPtr wheel_msg) {
   flWheelSpeed= (wheel_msg-> front_left)/3.6;
   frWheelSpeed = (wheel_msg-> front_right)/3.6;
   rlWheelSpeed = (wheel_msg-> rear_left)/3.6;
   rrWheelSpeed = (wheel_msg-> rear_right)/3.6;
}
void TakeHome::steering_callback(raptor_dbw_msgs::msg::SteeringExtendedReport::ConstSharedPtr steering_msg) {
  primary_steering_angle = steering_msg-> primary_steering_angle_fbk;
  wheelAngle = (primary_steering_angle/15)*(3.14/180);
  slip_ratio_calcs();
}


void TakeHome::curvillinear_callback(const std_msgs::msg::Float32::SharedPtr curvillinear_msg) {
  std_msgs::msg::Float32 LapTime;
  curvillinearDistance = curvillinear_msg->data;
  std::vector<double> curvillinearDistances;
  curvillinearDistances.push_back(curvillinearDistance);


   if (!curvillinearDistances.empty() && curvillinearDistance < .03 ) {
      rclcpp::Time now = this->get_clock()->now();
      LapTime.data = (now - last_lap_time_).seconds();
     RCLCPP_INFO(this->get_logger(), "imu_timestamps_ size 93: %zu", curvillinearDistances.size()); //debug


     last_lap_time_ = now;


     lapTime_publisher_->publish(LapTime);
   }
 
 
}


void TakeHome::imu_callback(novatel_oem7_msgs::msg::RAWIMU::ConstSharedPtr imu_msg) {


  //RCLCPP_INFO(this->get_logger(), "got in: %f", 30.0); debug
  std_msgs::msg::Float32 jitter_msg;
 
  rclcpp::Time msg_time(imu_msg->header.stamp);
 
  std::vector<double> deltaTimes;


  imu_timestamps_.push_back(msg_time);
 // RCLCPP_INFO(this->get_logger(), "imu_timestamps_ size 93: %zu", imu_timestamps_.size()); //debug




  while (!imu_timestamps_.empty() && (msg_time - imu_timestamps_.front()).seconds() > 1) {
    imu_timestamps_.erase(imu_timestamps_.begin());  // Erase the front element
  }
  //RCLCPP_INFO(this->get_logger(), "imu_timestamps_ size: %zu", imu_timestamps_.size()); //debug


  if (imu_timestamps_.size() < 2) {  //checks if there's things to compare against each other
       // did this happen
      // RCLCPP_INFO(this->get_logger(), "size less than 2: %f", 34.0); //debug
        return;
      }
  for (size_t i = 1; i < imu_timestamps_.size(); ++i)
    {
      double delta_t = (imu_timestamps_[i] - imu_timestamps_[i - 1]).seconds();
      deltaTimes.push_back(delta_t);
   
   //   RCLCPP_INFO(this->get_logger(), "float delta: %f", delta_t); //debug
      deltaTimes.push_back(delta_t);
  }


//with the delta_t values: now find the mean, and then calculate the variance


  double sum_deltaTimes =0;
  for (double t: deltaTimes){
    sum_deltaTimes += t;
  }


  if (deltaTimes.empty()) {
 //   RCLCPP_INFO(this->get_logger(), "125: %f", 390.0); //debug
   
    return;
  }


  double avg = sum_deltaTimes/deltaTimes.size();
  double variance = 0;
  for (double t: deltaTimes){
    variance += (t - avg) * (t - avg);
  }


  variance = variance/deltaTimes.size();
 
  jitter_msg.data = variance;
  // RCLCPP_INFO(this->get_logger(), "Received IMU at time: %f", imu_msg->header.stamp.sec + imu_msg->header.stamp.nanosec * 1e-9); debug
  RCLCPP_INFO(this->get_logger(), "variance: %f", variance);
  jitter_publisher_->publish(jitter_msg);
 


//   if (!curvillinearDistances.empty() && curvillinearDistance < .03 ) {
//     rclcpp::Time imu_time(imu_msg->header.stamp);
//     LapTime.data = (imu_time - last_lap_time_).seconds();
//     last_lap_time_= imu_time;
//     //RCLCPP_INFO(this->get_logger(), "imu_timestamps_ size 93: %zu", curvillinearDistances.size()); //debug
//     lapTime_publisher_->publish(LapTime);
//  }
 
 }




void TakeHome::slip_ratio_calcs() {
  std_msgs::msg::Float32 slipRR;
  std_msgs::msg::Float32 slipRL;
  std_msgs::msg::Float32 slipFL;
  std_msgs::msg::Float32 slipFR;


  slipRR.data = (rrWheelSpeed - (linearVx - 0.5*yaw*wr))/(linearVx - 0.5*yaw*wr);
  slip_rr_publisher_->publish(slipRR);
  slipRL.data = (rlWheelSpeed - (linearVx+0.5*yaw*wr))/(linearVx+0.5*yaw*wr);
  slip_rl_publisher_->publish(slipRL);
 
  //front left wheel slip ratio calc
  float vx_fl = linearVx+(0.5*yaw*wf);
  float vy_fl = linearVy+(yaw*lf);
  float vx_delta_fl = cos(wheelAngle)*vx_fl-sin(wheelAngle)*vy_fl;
  slipFL.data = (flWheelSpeed-vx_delta_fl)/vx_delta_fl;
  slip_fl_publisher_->publish(slipFL);


  //front right wheel slip ratio calc
  float vx_fr = linearVx-(0.5*yaw*wf);
  float vx_delta_fr = cos(wheelAngle)*vx_fr-sin(wheelAngle)*vy_fl;
  slipFR.data = (frWheelSpeed-vx_delta_fr)/vx_delta_fr;
  slip_fr_publisher_->publish(slipFR);
}




RCLCPP_COMPONENTS_REGISTER_NODE(TakeHome)

