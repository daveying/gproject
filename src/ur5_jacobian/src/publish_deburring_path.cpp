/**
 * # Description
 *   Publish a serial of frames and corresponding velocities respect to /world frame.
 *   TODO accept a srv call(or just a topic) to change the position of circle centor.
 *
 * # Topic out
 *   /path_vel(geometry_msgs::TwistStamped) current velocity of goal movement, this will feedforward to arm controller
 *   /goal_pose(geometry_msgs::PoseStamped) current pose of goal.
 *
 * # Broadcasted tf
 *   /world frame to /goal frame
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
vector<geometry_msgs::PoseStamped> generateLinePath(double length, double v_ampli, double freq);
void normalize(geometry_msgs::Vector3 &src, double len = 1);
vector<geometry_msgs::PoseStamped> generateDeburPath(double x, double y, double v_ampli, double freq);
void moveVel(ros::Publisher &pub, geometry_msgs::Twist msg_goal, geometry_msgs::Twist msg_cur, double sec = 1);

double offset = 0.027;

int main(int argc, char **argv)
{
    circle_centor.x = 0.5;
    circle_centor.y = -0.7;
    circle_centor.z = 0.945;//FIXME
    
    
    ros::init(argc, argv, "path_publisher");
    ros::NodeHandle n;

    tf::TransformBroadcaster br;
    ros::Publisher pub_pose = n.advertise<geometry_msgs::PoseStamped>("/goal_pose", 1000);
    ros::Publisher pub_agv = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

    vector<geometry_msgs::PoseStamped> path;
    vector<geometry_msgs::PoseStamped> path2;

    path = generateDeburPath(circle_centor.x, circle_centor.y, v_ampli, 30);
    path2 = generateDeburPath(0.74, circle_centor.y + offset, v_ampli, 30);

    ROS_INFO("test, %lf", path[0].pose.position.x);
    ROS_INFO("test, %lf", path2[0].pose.position.x);
    tf::Transform transform;
    tf::Quaternion orien;
    orien.setRPY(0, 0, 0);
    transform.setRotation(orien);
    transform.setOrigin(tf::Vector3(path[0].pose.position.x, path[0].pose.position.y, path[0].pose.position.z));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "goal"));
    pub_pose.publish(path[0]);
    

    
    ros::Rate rate(100);
    ros::Time begin_t = ros::Time::now();
    long index = 1;
    long index2 = 1;
    bool first_finish = false;
    while(ros::ok() && !first_finish)
    {
        ros::Time current = ros::Time::now();
        pub_pose.publish(path[index]);
        
        if(  (current.toSec() - begin_t.toSec() >= path[index - 1].header.stamp.toSec())   &&   index < path.size())
        {
            transform.setOrigin(tf::Vector3(path[index].pose.position.x, path[index].pose.position.y, path[index].pose.position.z));
            index++;
        }
        if(index == path.size())
        {
            first_finish = true;
        }

        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "goal"));
        rate.sleep();
        ros::spinOnce();
    }
    geometry_msgs::Twist agv_cur;
    geometry_msgs::Twist agv_goal;
    agv_cur.linear.x = 0;
    agv_cur.linear.y = 0;
    agv_goal.linear.x = 1;
    agv_goal.linear.y = 0;
    moveVel(pub_agv, agv_goal, agv_cur);
    ros::Duration(4).sleep();
    agv_cur.linear.x = 1;
    agv_cur.linear.y = 0.0;
    agv_goal.linear.x = 0;
    agv_goal.linear.y = 0;
    moveVel(pub_agv, agv_goal, agv_cur);
    begin_t = ros::Time::now();
    while(ros::ok())
    {
        ros::Time current = ros::Time::now();
        pub_pose.publish(path2[index2]);
        
        if(  (current.toSec() - begin_t.toSec() >= path2[index2 - 1].header.stamp.toSec())   &&   index2 < path2.size())
        {
            transform.setOrigin(tf::Vector3(path2[index2].pose.position.x, path2[index2].pose.position.y, path2[index2].pose.position.z));
            index2++;
        }
        else
        {
            first_finish = true;
        }

        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "goal"));
        pub_agv.publish(agv_goal);
        rate.sleep();
        ros::spinOnce();
    }
    ros::spin();
}


void moveVel(ros::Publisher &pub, geometry_msgs::Twist msg_goal, geometry_msgs::Twist msg_cur, double sec)
{
    int sec_int = sec * 10;
    geometry_msgs::Twist msg;
    for(int i = 0; i < sec_int; i++)
    {
        msg.linear.x = msg_cur.linear.x + (msg_goal.linear.x - msg_cur.linear.x) * i / sec_int;
        msg.linear.y = msg_cur.linear.y + (msg_goal.linear.y - msg_cur.linear.y) * i / sec_int;
        msg.angular.z = msg_cur.angular.z + (msg_goal.angular.z - msg_cur.angular.z) * i / sec_int;
        
        pub.publish(msg);
        ros::Duration(0.1).sleep();
    }

}

void normalize(geometry_msgs::Vector3 &src, double len)
{
    double length = sqrt(src.x * src.x + src.y * src.y + src.z * src.z);
    src.x = src.x * len / length;
    src.y = src.y * len / length;
    src.z = src.z * len / length;
}

vector<geometry_msgs::PoseStamped> generateDeburPath(double x, double y, double v_ampli, double freq)
{    
    geometry_msgs::PoseStamped item;
    vector<geometry_msgs::PoseStamped> path;
    
    item.header.frame_id = "/world";
    item.header.stamp = ros::Time(3); //FIXME, choose a suitable duration
    item.pose.position.x = x;
    item.pose.position.y = y;
    item.pose.position.z = circle_centor.z + 0.03;
    item.pose.orientation.w = 1;
    item.pose.orientation.x = item.pose.orientation.y = item.pose.orientation.z = 0;
    
    path.push_back(item);
    
    item.pose.position.z = circle_centor.z;
    item.header.stamp += ros::Duration(1);
    path.push_back(item);
    
    double long_time = 4;
    double short_time = 2;
    double stay_time = 2;
    
    for(int i = 0; i < long_time * freq; i++)
    {
        item.pose.position.x += v_ampli / freq;
        item.header.stamp += ros::Duration(1/freq);
        path.push_back(item);
    }
    path[path.size() - 1].header.stamp += ros::Duration(stay_time);
    for(int i = 0; i < short_time * freq; i++)
    {
        item.pose.position.y += v_ampli / freq;
        item.header.stamp += ros::Duration(1/freq);
        path.push_back(item);
    }
    path[path.size() - 1].header.stamp += ros::Duration(stay_time);
    for(int i = 0; i < short_time * freq; i++)
    {
        item.pose.position.x += v_ampli / freq;
        item.header.stamp += ros::Duration(1/freq);
        path.push_back(item);
    }
    path[path.size() - 1].header.stamp += ros::Duration(stay_time);
    for(int i = 0; i < short_time * freq; i++)
    {
        item.pose.position.y -= v_ampli / freq;
        item.header.stamp += ros::Duration(1/freq);
        path.push_back(item);
    }
    path[path.size() - 1].header.stamp += ros::Duration(stay_time);
    item.pose.position.z += 0.03;
    item.header.stamp += ros::Duration(1);
    path.push_back(item);
    
    if(x == circle_centor.x && y == circle_centor.y)
    {
        item.pose.position.y += offset;
    }
    
    return path;
    
}

vector<geometry_msgs::PoseStamped> generateCirclePath(double radius, double v_ampli, double freq)
{
    geometry_msgs::PoseStamped item;
    vector<geometry_msgs::PoseStamped> path;
    
    item.header.frame_id = "/world";
    item.header.stamp = ros::Time(5); //FIXME, choose a suitable duration
    item.pose.position.x = circle_centor.x + radius;
    item.pose.position.y = circle_centor.y;
    item.pose.position.z = circle_centor.z + 0.03;
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

vector<geometry_msgs::PoseStamped> generateLinePath(double length, double v_ampli, double freq)
{
    geometry_msgs::PoseStamped item;
    vector<geometry_msgs::PoseStamped> path;
    
    item.header.frame_id = "/world";
    item.header.stamp = ros::Time(5); //FIXME, choose a suitable duration
    item.pose.position.x = circle_centor.x;
    item.pose.position.y = circle_centor.y + length / 2;
    item.pose.position.z = circle_centor.z + 0.1;
    item.pose.orientation.w = 1;
    item.pose.orientation.x = item.pose.orientation.y = item.pose.orientation.z = 0;
    
    path.push_back(item);
    
    item.pose.position.z = circle_centor.z;
    item.header.stamp += ros::Duration(1);
    path.push_back(item);
    
    double delta_x = v_ampli / freq;
    for(int i = 0; i <= length / delta_x; i++)
    {
        item.header.stamp += ros::Duration(1 / freq);
        item.pose.position.y = circle_centor.y + length / 2 - delta_x * i;
        path.push_back(item);
    }
    
    path[path.size() - 1].header.stamp += ros::Duration(1);
    item.pose.position.z = circle_centor.z + 0.05;
    path.push_back(item);
    return path;
}







