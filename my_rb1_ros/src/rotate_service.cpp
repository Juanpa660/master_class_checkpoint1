#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ros/subscriber.h"
#include <cmath>
#include <nav_msgs/Odometry.h>
#include <my_rb1_ros/Rotate.h>
#include <unistd.h>

ros::Publisher vel_pub;
ros::Subscriber odom_sub;
geometry_msgs::Twist vel_msg;
double current_orientation = 0.0;
using namespace std;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // Extract the orientation from the odometry message
    double x = msg->pose.pose.orientation.x;
    double y = msg->pose.pose.orientation.y;
    double z = msg->pose.pose.orientation.z;
    double w = msg->pose.pose.orientation.w;

    // Convert quaternion to yaw angle (z-axis rotation)
    current_orientation = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z));
}

bool my_callback(my_rb1_ros::Rotate::Request  &req,
                 my_rb1_ros::Rotate::Response &res)
{  
  ROS_INFO("The Service rotate_robot has been called");

  // Target Orientation
  double target_orientation = current_orientation + req.degrees * M_PI / 180.0;
  ros::Rate rate(10); // Adjust the frequency as needed
  while (std::abs(target_orientation - current_orientation) > 0.05)
  {
    // ROS_INFO("Target Orientation: %f, Current Orientation: %f, Diff: %f", target_orientation*(180.0/M_PI), current_orientation*(180.0/M_PI), (target_orientation - current_orientation)*(180.0/M_PI));

    if (req.degrees > 0)
    {
    vel_msg.angular.z = 0.2;
    } else {
    vel_msg.angular.z = -0.2;
    }
    vel_pub.publish(vel_msg);
    ros::spinOnce();
    rate.sleep();
  }
  res.result = true;
  vel_msg.angular.z = 0.0;
  vel_pub.publish(vel_msg);
  ROS_INFO("Finished service rotate_robot");
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rotate_server");
  ros::NodeHandle nh;

  ros::ServiceServer my_service = nh.advertiseService("/rotate_robot", my_callback);
  
  odom_sub = nh.subscribe("/odom", 1000, odomCallback);
  vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  ROS_INFO("Service /rotate_robot Ready");
  ros::spin();

  return 0;
}