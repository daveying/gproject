#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float64.h>
#include "geometry_msgs/Twist.h"


int count = 0;
bool beginn = false;

void statusCallback(const std_msgs::Int8::ConstPtr& msg);
void moveVel(ros::Publisher &pub, geometry_msgs::Twist msg_goal, geometry_msgs::Twist msg_cur, double sec = 1);

int main(int argc, char** argv)
{
	ros::init(argc, argv, "yaskawa_demo");
	ros::NodeHandle n;
	
	ros::Publisher pub_agv = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

	geometry_msgs::Twist agv_cur;
	geometry_msgs::Twist agv_goal;
	
	agv_cur.linear.x = 0;
	agv_cur.linear.y = 0;
	agv_goal.linear.x = 0.5;
	agv_goal.linear.y = 0;
	moveVel(pub_agv, agv_goal, agv_cur);
	ros::Duration(25).sleep();
	agv_cur.linear.x = 0.5;
	agv_cur.linear.y = 0.0;
	agv_goal.linear.x = 0;
	agv_goal.linear.y = 0;
	moveVel(pub_agv, agv_goal, agv_cur);
	
	

	ros::Rate r(10);
	
	while(ros::ok())
	{
	    agv_goal.linear.x = 0;
	    agv_goal.linear.y = 0;
	    pub_agv.publish(agv_goal);
	    r.sleep();
	    ros::spinOnce();
	}
	
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

