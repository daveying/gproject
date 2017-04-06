#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float64.h>
#include "geometry_msgs/Twist.h"


int count = 0;
bool beginn = false;

void statusCallback(const std_msgs::Int8::ConstPtr& msg);

int main(int argc, char** argv)
{
	ros::init(argc, argv, "yaskawa_demo");
	ros::NodeHandle n;
	
	ros::Publisher pub_agv = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

	geometry_msgs::Twist agv_msg;
	agv_msg.linear.x = 0.0;
	double t = 3;
	double vel = 0.25;
	    //y increace vel
		agv_msg.linear.y += 0.1;
		pub_agv.publish(agv_msg);
		ros::Duration(0.1).sleep();
		agv_msg.linear.y += 0.1;
		pub_agv.publish(agv_msg);
		ros::Duration(0.1).sleep();
		agv_msg.linear.y += vel;
		pub_agv.publish(agv_msg);
		ros::Duration(0.1).sleep();
		agv_msg.linear.y += vel;
		pub_agv.publish(agv_msg);
		ros::Duration(0.1).sleep();
		agv_msg.linear.y += vel;
		pub_agv.publish(agv_msg);
		ros::Duration(0.1).sleep();
		agv_msg.linear.y += vel;
		pub_agv.publish(agv_msg);
		ros::Duration(0.1).sleep();
		agv_msg.linear.y += 0.1;
		pub_agv.publish(agv_msg);
		
		ros::Duration(t-1).sleep();
		
	//y decreace vel
		agv_msg.linear.y -= 0.1;
		pub_agv.publish(agv_msg);
		ros::Duration(0.1).sleep();
		agv_msg.linear.y -= vel;
		pub_agv.publish(agv_msg);
		ros::Duration(0.1).sleep();
		agv_msg.linear.y -= vel;
		pub_agv.publish(agv_msg);
		ros::Duration(0.1).sleep();
		agv_msg.linear.y -= vel;
		pub_agv.publish(agv_msg);
		ros::Duration(0.1).sleep();
		agv_msg.linear.y -= vel;
		pub_agv.publish(agv_msg);
		ros::Duration(0.1).sleep();
		agv_msg.linear.y -= 0.1;
		pub_agv.publish(agv_msg);
		ros::Duration(0.1).sleep();
		agv_msg.linear.y -= 0.1;
		pub_agv.publish(agv_msg);
		
		
	    //x increace vel
		agv_msg.linear.x += 0.1;
		pub_agv.publish(agv_msg);
		ros::Duration(0.1).sleep();
		agv_msg.linear.x += 0.1;
		pub_agv.publish(agv_msg);
		ros::Duration(0.1).sleep();
		agv_msg.linear.x += vel;
		pub_agv.publish(agv_msg);
		ros::Duration(0.1).sleep();
		agv_msg.linear.x += vel;
		pub_agv.publish(agv_msg);
		ros::Duration(0.1).sleep();
		agv_msg.linear.x += vel;
		pub_agv.publish(agv_msg);
		ros::Duration(0.1).sleep();
		agv_msg.linear.x += vel;
		pub_agv.publish(agv_msg);
		ros::Duration(0.1).sleep();
		agv_msg.linear.x += 0.1;
		pub_agv.publish(agv_msg);
		
		ros::Duration(t).sleep();
		
	//x decreace vel
		agv_msg.linear.x -= 0.1;
		pub_agv.publish(agv_msg);
		ros::Duration(0.1).sleep();
		agv_msg.linear.x -= vel;
		pub_agv.publish(agv_msg);
		ros::Duration(0.1).sleep();
		agv_msg.linear.x -= vel;
		pub_agv.publish(agv_msg);
		ros::Duration(0.1).sleep();
		agv_msg.linear.x -= vel;
		pub_agv.publish(agv_msg);
		ros::Duration(0.1).sleep();
		agv_msg.linear.x -= vel;
		pub_agv.publish(agv_msg);
		ros::Duration(0.1).sleep();
		agv_msg.linear.x -= 0.1;
		pub_agv.publish(agv_msg);
		ros::Duration(0.1).sleep();
		agv_msg.linear.x -= 0.1;
		pub_agv.publish(agv_msg);

	//y decreace vel
		agv_msg.linear.y -= 0.1;
		pub_agv.publish(agv_msg);
		ros::Duration(0.1).sleep();
		agv_msg.linear.y -= vel;
		pub_agv.publish(agv_msg);
		ros::Duration(0.1).sleep();
		agv_msg.linear.y -= vel;
		pub_agv.publish(agv_msg);
		ros::Duration(0.1).sleep();
		agv_msg.linear.y -= vel;
		pub_agv.publish(agv_msg);
		ros::Duration(0.1).sleep();
		agv_msg.linear.y -= vel;
		pub_agv.publish(agv_msg);
		ros::Duration(0.1).sleep();
		agv_msg.linear.y -= 0.1;
		pub_agv.publish(agv_msg);
		ros::Duration(0.1).sleep();
		agv_msg.linear.y -= 0.1;
		pub_agv.publish(agv_msg);
		
		ros::Duration(t-1).sleep();
		
	 //y increace vel
		agv_msg.linear.y += 0.1;
		pub_agv.publish(agv_msg);
		ros::Duration(0.1).sleep();
		agv_msg.linear.y += 0.1;
		pub_agv.publish(agv_msg);
		ros::Duration(0.1).sleep();
		agv_msg.linear.y += vel;
		pub_agv.publish(agv_msg);
		ros::Duration(0.1).sleep();
		agv_msg.linear.y += vel;
		pub_agv.publish(agv_msg);
		ros::Duration(0.1).sleep();
		agv_msg.linear.y += vel;
		pub_agv.publish(agv_msg);
		ros::Duration(0.1).sleep();
		agv_msg.linear.y += vel;
		pub_agv.publish(agv_msg);
		ros::Duration(0.1).sleep();
		agv_msg.linear.y += 0.1;
		pub_agv.publish(agv_msg);

	//x decreace vel
		agv_msg.linear.x -= 0.1;
		pub_agv.publish(agv_msg);
		ros::Duration(0.1).sleep();
		agv_msg.linear.x -= vel;
		pub_agv.publish(agv_msg);
		ros::Duration(0.1).sleep();
		agv_msg.linear.x -= vel;
		pub_agv.publish(agv_msg);
		ros::Duration(0.1).sleep();
		agv_msg.linear.x -= vel;
		pub_agv.publish(agv_msg);
		ros::Duration(0.1).sleep();
		agv_msg.linear.x -= vel;
		pub_agv.publish(agv_msg);
		ros::Duration(0.1).sleep();
		agv_msg.linear.x -= 0.1;
		pub_agv.publish(agv_msg);
		ros::Duration(0.1).sleep();
		agv_msg.linear.x -= 0.1;
		pub_agv.publish(agv_msg);
		
		ros::Duration(t).sleep();
		
			    //x increace vel
		agv_msg.linear.x += 0.1;
		pub_agv.publish(agv_msg);
		ros::Duration(0.1).sleep();
		agv_msg.linear.x += 0.1;
		pub_agv.publish(agv_msg);
		ros::Duration(0.1).sleep();
		agv_msg.linear.x += vel;
		pub_agv.publish(agv_msg);
		ros::Duration(0.1).sleep();
		agv_msg.linear.x += vel;
		pub_agv.publish(agv_msg);
		ros::Duration(0.1).sleep();
		agv_msg.linear.x += vel;
		pub_agv.publish(agv_msg);
		ros::Duration(0.1).sleep();
		agv_msg.linear.x += vel;
		pub_agv.publish(agv_msg);
		ros::Duration(0.1).sleep();
		agv_msg.linear.x += 0.1;
		pub_agv.publish(agv_msg);
		
		ros::Rate r(10);
		
		while(ros::ok())
		{
		    agv_msg.linear.x = 0;
		    agv_msg.linear.y = 0;
		    pub_agv.publish(agv_msg);
		    r.sleep();
		ros::spinOnce();
		}
		
	
	
}

