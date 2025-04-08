#include "ros/console.h"
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

class Joy2Cmd final {
public:
  Joy2Cmd() {
    // Load parameters if needed
    ros::NodeHandle private_nh("~");
    private_nh.param("linear_axis_x", linear_axis_x_, 1);  // usually left stick vertical
    private_nh.param("linear_axis_y", linear_axis_y_, 3);  // usually left stick vertical
    private_nh.param("angular_axis", angular_axis_, 0);  // usually left stick horizontal
    private_nh.param("scale_linear", scale_linear_, 0.25);
    private_nh.param("scale_angular", scale_angular_, 0.25);

    joy_sub_ = nh_.subscribe("joy", 10, &Joy2Cmd::joyCallback, this);
    cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  }

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
    geometry_msgs::Twist cmd;
    cmd.linear.x = scale_linear_ * joy->axes[linear_axis_x_];
    cmd.linear.y = scale_linear_ * joy->axes[linear_axis_y_];
    cmd.angular.z = scale_angular_ * joy->axes[angular_axis_];
    cmd_pub_.publish(cmd);
    ROS_INFO("Publishing cmd_vel: linear = (%f, %f), angular.z = %f",
      cmd.linear.x, 
      cmd.linear.y, 
      cmd.angular.z);
  }

  ros::NodeHandle nh_;
  ros::Subscriber joy_sub_;
  ros::Publisher cmd_pub_;
  int linear_axis_x_, linear_axis_y_, angular_axis_;
  double scale_linear_, scale_angular_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "joy2cmd_cpp");
  Joy2Cmd joy2cmd;
  ros::spin();
}
