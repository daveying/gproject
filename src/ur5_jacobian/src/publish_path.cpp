/**
 * # Description
 *   Publish a serial of frames and corresponding velocities respect to /world frame
 *
 * # Run this node with
 * $ rosrun ur5_jacobian publish_path
 */
 
#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include <vector>
using namespace std;

const double v_ampli = 0.04; //unit m/s. amplitude of tcp moving velocity
const double pi = 3.1415926;
geometry_msgs::Point circle_centor;

vector<geometry_msgs::PoseStamped> generateCirclePath(double radius, double v_ampli, double freq);
void normalize(geometry_msgs::Vector3 &src, double len = 1);

int main(int argc, char **argv)
{
    circle_centor.x = 0.5;
    circle_centor.y = -0.5;
    circle_centor.z = 0.8;//TODO
    
    
    ros::init(argc, argv, "path_publisher");
    ros::NodeHandle n;

    tf::TransformBroadcaster br;
    ros::Publisher pub_path_vel = n.advertise<geometry_msgs::TwistStamped>("/path_vel", 1000);
    ros::Publisher pub_pose = n.advertise<geometry_msgs::PoseStamped>("/goal_pose", 1000);
    
    vector<geometry_msgs::PoseStamped> path = generateCirclePath(0.1, v_ampli, 30);
    ROS_INFO("test, %lf", path[0].pose.position.x);
    tf::Transform transform;
    tf::Quaternion orien;
    orien.setRPY(0, 0, 0);
    transform.setRotation(orien);
    transform.setOrigin(tf::Vector3(path[0].pose.position.x, path[0].pose.position.y, path[0].pose.position.z));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "goal"));
    pub_pose.publish(path[0]);
    
    geometry_msgs::TwistStamped vel_msg;
    vel_msg.header.stamp = ros::Time::now();
    vel_msg.twist.linear.x = 0;
    vel_msg.twist.linear.y = 0;
    vel_msg.twist.linear.z = 0;
    vel_msg.twist.angular.x = 0;
    vel_msg.twist.angular.y = 0;
    vel_msg.twist.angular.z = 0;
    pub_path_vel.publish(vel_msg);
    
    ros::Rate rate(30);
    ros::Time begin_t = ros::Time::now();
    long index = 1;
    while(ros::ok())
    {
        ros::Time current = ros::Time::now();
        pub_pose.publish(path[index]);
        
        if(  (current.toSec() - begin_t.toSec() >= path[index - 1].header.stamp.toSec())   &&   index < path.size())
        {
            transform.setOrigin(tf::Vector3(path[index].pose.position.x, path[index].pose.position.y, path[index].pose.position.z));
            vel_msg.twist.linear.x = -path[index].pose.position.y;
            vel_msg.twist.linear.y = path[index].pose.position.x;
            normalize(vel_msg.twist.linear, v_ampli);
            index++;
        }
        pub_path_vel.publish(vel_msg);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "goal"));
        rate.sleep();
        ros::spinOnce();
    }
}

void normalize(geometry_msgs::Vector3 &src, double len)
{
    double length = sqrt(src.x * src.x + src.y * src.y + src.z * src.z);
    src.x = src.x * len / length;
    src.y = src.y * len / length;
    src.z = src.z * len / length;
}

vector<geometry_msgs::PoseStamped> generateCirclePath(double radius, double v_ampli, double freq)
{
    geometry_msgs::PoseStamped item;
    vector<geometry_msgs::PoseStamped> path;
    
    item.header.frame_id = "/world";
    item.header.stamp = ros::Time(3); //TODO, choose a suitable duration
    item.pose.position.x = circle_centor.x + radius;
    item.pose.position.y = circle_centor.y;
    item.pose.position.z = circle_centor.z + 0.05;
    item.pose.orientation.w = 1;
    item.pose.orientation.x = item.pose.orientation.y = item.pose.orientation.z = 0;
    
    path.push_back(item);
    
    item.pose.position.z = circle_centor.z;
    item.header.stamp += ros::Duration(1);
    path.push_back(item);
    
    double delt_theta = v_ampli / (radius * freq);
    for(int i = 1; i <= 2 * pi / delt_theta; i++)
    {
        item.header.stamp += ros::Duration(1 / freq);
        item.pose.position.x = circle_centor.x + radius * cos(i*delt_theta);
        item.pose.position.y = circle_centor.y + radius * sin(i*delt_theta);
        path.push_back(item);
    }
    path[path.size() - 1].header.stamp += ros::Duration(1);
    item = path[0];
    item.header.stamp = path[path.size() - 1].header.stamp + ros::Duration(1);
    path.push_back(item);
    return path;
}







