/**
 * # Description
 *   Arm PID + feedforward controller.
 *   Input: Pose difference between /reference frame and /goal frame, moving velocity of goal point
 *   Output: tcp velocities respect to /world frame
 *   
 * # Topic in
 *   /path_vel(geometry_msgs::TwistStamped) moving velocity of goal point for feedforward control
 *   /tcp_velocities(geometry_msgs/TwistStamped) actual velocity of tcp. For D control
 *
 * # Topic out
 *   /tcp_velocities(geometry_msgs/TwistStamped) for diagnose
 *
 *
 * # run this node
 * $ rosrun ur5_jacobian arm_controller
 */
 
#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "geometry_msgs/TwistStamped.h"
#include <Eigen/Dense>

Eigen::MatrixXd path_vel(6, 1);
Eigen::MatrixXd tcp_vel(6, 1);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_controller");
    ros::NodeHandle n;
    
    ros::Subscriber sub_path_vel = n.subscribe("/path_vel", 1000, pathVelCallback);
    ros::Subscriber sub_tcp_vel = n.subscribe("/tcp_velocities", 1000, tcpVelCallback);
    
    tf::TransformListener tf_listener;
}

void pathVelCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    path_vel(0, 0) = msg->twist.linear.x;
    path_vel(1, 0) = msg->twist.linear.y;
    path_vel(2, 0) = msg->twist.linear.z;
    path_vel(3, 0) = msg->twist.angular.x;
    path_vel(4, 0) = msg->twist.angular.y;
    path_vel(5, 0) = msg->twist.angular.z;
}

void tcpVelCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    tcp_vel(0, 0) = msg->twist.linear.x;
    tcp_vel(1, 0) = msg->twist.linear.y;
    tcp_vel(2, 0) = msg->twist.linear.z;
    tcp_vel(3, 0) = msg->twist.angular.x;
    tcp_vel(4, 0) = msg->twist.angular.y;
    tcp_vel(5, 0) = msg->twist.angular.z;
}
