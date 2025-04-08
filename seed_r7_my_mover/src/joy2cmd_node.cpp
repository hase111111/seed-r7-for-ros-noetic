
#include <mutex>

#include "ros/console.h"
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

class Joy2Cmd final {
public:
  Joy2Cmd() {
    // Load parameters if needed
    ros::NodeHandle private_nh("~");
    private_nh.param("linear_axis_x", linear_axis_x_, 1);
    private_nh.param("linear_axis_y", linear_axis_y_, 3);
    private_nh.param("angular_axis", angular_axis_, 0);
    private_nh.param("scale_linear", scale_linear_, 0.25);
    private_nh.param("scale_angular", scale_angular_, 0.25);

    joy_sub_ = nh_.subscribe("joy", 10, &Joy2Cmd::joyCallback, this);
    cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    publish_timer_ = nh_.createTimer(ros::Duration(0.1), &Joy2Cmd::publishCmdVel, this);  // 10Hz
  }

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
    std::lock_guard<std::mutex> lock(mtx_);

    latest_cmd_.linear.x = scale_linear_ * joy->axes[linear_axis_x_];
    latest_cmd_.linear.y = scale_linear_ * joy->axes[linear_axis_y_];
    latest_cmd_.angular.z = scale_angular_ * joy->axes[angular_axis_];

    ROS_INFO("Publishing cmd_vel: linear = (%f, %f), angular.z = %f",
      latest_cmd_.linear.x, 
      latest_cmd_.linear.y, 
      latest_cmd_.angular.z);
  }

  void publishCmdVel(const ros::TimerEvent&) {
    std::lock_guard<std::mutex> lock(mtx_);

    cmd_pub_.publish(latest_cmd_);

    // 減速
    constexpr double deceleration_factor = 0.95;
    latest_cmd_.linear.x *= deceleration_factor;
    latest_cmd_.linear.y *= deceleration_factor;
    latest_cmd_.angular.z *= deceleration_factor;
  }

  ros::NodeHandle nh_;
  ros::Subscriber joy_sub_;
  ros::Publisher cmd_pub_;
  ros::Timer publish_timer_;
  int linear_axis_x_, linear_axis_y_, angular_axis_;
  double scale_linear_, scale_angular_;
  geometry_msgs::Twist latest_cmd_;
  std::mutex mtx_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "joy2cmd_cpp");
  Joy2Cmd joy2cmd;
  ros::spin();
}
