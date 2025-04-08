
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

    latest_cmd_.linear.x = latest_cmd_.linear.y = latest_cmd_.angular.z = 0.0;
  }

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
    std::lock_guard<std::mutex> lock(mtx_);

    axes_x = scale_linear_ * joy->axes[linear_axis_x_];
    axes_y = scale_linear_ * joy->axes[linear_axis_y_];
    axes_z = scale_angular_ * joy->axes[angular_axis_];

    ROS_INFO("axes_x: %f, axes_y: %f, axes_z: %f", axes_x, axes_y, axes_z);
  }

  void publishCmdVel(const ros::TimerEvent&) {
    std::lock_guard<std::mutex> lock(mtx_);

    updateCmdVel();

    cmd_pub_.publish(latest_cmd_);
  }

  void updateCmdVel() {
    constexpr double rate = 0.1;

    latest_cmd_.linear.x = axes_x * rate + latest_cmd_.linear.x * (1 - rate);
    latest_cmd_.linear.y = axes_y * rate + latest_cmd_.linear.y * (1 - rate);
    latest_cmd_.angular.z = axes_z * rate + latest_cmd_.angular.z * (1 - rate);
  }

  ros::NodeHandle nh_;
  ros::Subscriber joy_sub_;
  ros::Publisher cmd_pub_;
  ros::Timer publish_timer_;
  int linear_axis_x_, linear_axis_y_, angular_axis_;  // const
  double axes_x{0.0}, axes_y{0.0}, axes_z{0.0};
  double scale_linear_, scale_angular_;
  geometry_msgs::Twist latest_cmd_;
  std::mutex mtx_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "joy2cmd_cpp");
  Joy2Cmd joy2cmd;
  ros::spin();
}
