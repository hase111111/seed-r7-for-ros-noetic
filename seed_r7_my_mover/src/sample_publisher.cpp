#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cmd_vel_publisher");
    ros::NodeHandle nh;

    // Publisher
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // Loop rate in Hz
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        geometry_msgs::Twist msg;
        msg.linear.x = 0.2;   // Move forward
        msg.angular.z = 0.5;  // Rotate

        vel_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
