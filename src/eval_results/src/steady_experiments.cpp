#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "geometry_msgs/TwistStamped.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "sensor_msgs/JointState.h"
#include <iostream>
#include <fstream>
using namespace std;

tf::Vector3 platform_linear_vel;
tf::Vector3 platform_angular_vel;
tf::Vector3 tcp_linear_vel;
tf::Vector3 tcp_angular_vel;

vector<double> trj_command(6, 0.0);
vector<double> trj_actual(6, 0.0);
vector<double> trj_position(6, 0.0);

tf::Vector3 path_vel;

void updateTransform(tf::TransformListener &tf_listener, tf::StampedTransform &ub_w_transform, string parent, string child);
void platformVelCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
void pathVelCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
void tcpVelCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
void trjCallback(const trajectory_msgs::JointTrajectory::ConstPtr& msg);
void trjActualCallback(const sensor_msgs::JointState::ConstPtr& msg);

int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "get_pose");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);
    
    tf::TransformListener tf_listener;
    tf::StampedTransform ref_w_transform;
    tf::StampedTransform g_w_transform;
    tf::StampedTransform urbase_w_transform;
    tf::StampedTransform base_w_transform;
    
    ros::Subscriber sub_platform_vel = n.subscribe("/laserplatform_speeds", 1000, platformVelCallback);
    ros::Subscriber sub_path_vel = n.subscribe("/path_vel", 1000, pathVelCallback);
    ros::Subscriber sub_tcp_vel = n.subscribe("/tcp_velocities", 1000, tcpVelCallback);
    ros::Subscriber sub_trj_command = n.subscribe("/ur_driver/joint_speed", 1000, trjCallback);
    ros::Subscriber sub_trj_actual = n.subscribe("/joint_states", 1000, trjActualCallback);
    
    ofstream datafile;
    datafile.open("data.txt");
    
    while(ros::ok())
    {
        updateTransform(tf_listener, ref_w_transform, "/world", "/reference");
        updateTransform(tf_listener, g_w_transform, "/world", "/goal");
        updateTransform(tf_listener, urbase_w_transform, "/world", "/ur_base_link");
        updateTransform(tf_listener, base_w_transform, "/world", "/base_link");
        tf::Vector3 ref_origin = ref_w_transform.getOrigin();
        tf::Vector3 g_origin = g_w_transform.getOrigin();
        tf::Vector3 base_origin = base_w_transform.getOrigin();
        double base_yaw = base_w_transform.getRotation().getAngle();
        
        urbase_w_transform.setOrigin(tf::Vector3(0, 0, 0));
        tf::Vector3 tcp_vel_world = urbase_w_transform * tcp_linear_vel;
        
        // reference pose[3] -- goal pose[3] -- goal vel[2] -- platform vel[1] -- tcp vel[2] -- base pose[2] -- base heading[1] -- platform vel[2] -- joint command vel[6] -- joint actual vel[6]
        datafile << ref_origin[0] << " " << ref_origin[1] << " " << ref_origin[2] << " " << g_origin[0] << " " << g_origin[1] << " " << g_origin[2] << " " << path_vel[0] << " " << path_vel[1] << " " << platform_linear_vel[0] << " " << tcp_vel_world[0] << " " << tcp_vel_world[1] << " " << base_origin[0] << " " << base_origin[1] << " " << base_yaw << " " << platform_linear_vel[1] << " " << platform_angular_vel[2] << " " << trj_command[0] << " " << trj_command[1] << " " << trj_command[2] << " " << trj_command[3] << " " << trj_command[4] << " " << trj_command[5] << " " << trj_actual[0] << " " << trj_actual[1] << " " << trj_actual[2] << " " << trj_actual[3] << " " << trj_actual[4] << " " << trj_actual[5] << " " << trj_position[0] << " " << trj_position[1] << " " << trj_position[2] << " " << trj_position[3] << " " << trj_position[4] << " " << trj_position[5] << endl;
        loop_rate.sleep();
        ros::spinOnce();
    }
    ros::spin();
    datafile.close();
    
}

void updateTransform(tf::TransformListener &tf_listener, tf::StampedTransform &transform, string parent, string child)
{
    bool s = false;
    while(!s && ros::ok())
    {
        try
        {
            tf_listener.lookupTransform(parent, child, ros::Time(0), transform);
            s = true;
        }
        catch (tf::TransformException &ex)
        {
			ROS_ERROR("%s", ex.what());
			ros::Duration(0.01).sleep();
			continue;
        }
    }
}

void trjActualCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    if(msg->velocity.size() == 6)
    {
        trj_actual[0] = msg->velocity[0];
        trj_actual[1] = msg->velocity[1];
        trj_actual[2] = msg->velocity[2];
        trj_actual[3] = msg->velocity[3];
        trj_actual[4] = msg->velocity[4];
        trj_actual[5] = msg->velocity[5];
        trj_position[0] = msg->position[0];
        trj_position[1] = msg->position[1];
        trj_position[2] = msg->position[2];
        trj_position[3] = msg->position[3];
        trj_position[4] = msg->position[4];
        trj_position[5] = msg->position[5];
        
    }
}

void trjCallback(const trajectory_msgs::JointTrajectory::ConstPtr& msg)
{
        trj_command[0] = msg->points[0].velocities[0];
        trj_command[1] = msg->points[0].velocities[1];
        trj_command[2] = msg->points[0].velocities[2];
        trj_command[3] = msg->points[0].velocities[3];
        trj_command[4] = msg->points[0].velocities[4];
        trj_command[5] = msg->points[0].velocities[5];
}

void platformVelCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    platform_linear_vel[0] = msg->twist.linear.x;
    platform_linear_vel[1] = msg->twist.linear.y;
    platform_linear_vel[2] = msg->twist.linear.z;
    platform_angular_vel[0] = msg->twist.angular.x;
    platform_angular_vel[1] = msg->twist.angular.y;
    platform_angular_vel[2] = msg->twist.angular.z;
}

void pathVelCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    path_vel[0] = msg->twist.linear.x;
    path_vel[1] = msg->twist.linear.y;
    path_vel[2] = msg->twist.linear.z;
}

void tcpVelCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    tcp_linear_vel[0] = msg->twist.linear.x;
    tcp_linear_vel[1] = msg->twist.linear.y;
    tcp_linear_vel[2] = msg->twist.linear.z;
    tcp_angular_vel[0] = msg->twist.angular.x;
    tcp_angular_vel[1] = msg->twist.angular.y;
    tcp_angular_vel[2] = msg->twist.angular.z;
}
