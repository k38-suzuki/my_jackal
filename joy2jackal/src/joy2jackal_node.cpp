/**
   @author Kenta Suzuki
*/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

ros::Publisher vel_pub;

void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
    double k[] = { 0.0, 0.0 };
    if(msg->buttons[4]) {
        k[0] = 0.2;
        k[1] = 1.0;
    } else if(msg->buttons[5]) {
        k[0] = 1.0;
        k[1] = 1.0;
    }

    geometry_msgs::Twist twist;
    twist.linear.x = k[0] * 2.0 * msg->axes[1];
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = k[1] * 1.4 * msg->axes[0];
    vel_pub.publish(twist);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "joy2jackal");
    ros::NodeHandle n;
    ros::Subscriber joy_sub = n.subscribe("joy", 1, &joyCallback);
    vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    ros::spin();

    return 0;
}
