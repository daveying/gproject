#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Twist.h"
#include <tf/tf.h>
#include <tf/LinearMath/Matrix3x3.h>


int main(int argc, char **argv)
{
	ros::init(argc, argv, "getVelocity");
	ros::NodeHandle n;
	ros::Publisher velocity_pub = n.advertise<geometry_msgs::Twist>("velocity", 1000);
	tf::TransformListener listener;
	ros::Rate loop_rate(30);
	double duration_time = 0.03;
	int count = 0;
	geometry_msgs::Twist velocity;
	double current_x, current_y, last_x, last_y;
	tf::Matrix3x3 current_m, last_m;
	while(n.ok())
	{
		// get TF
		tf::StampedTransform transform;
		try{
		  		listener.lookupTransform("/world", "/base_link",  ros::Time(0), transform);
			}
		catch (tf::TransformException ex){
	  		ROS_ERROR("%s",ex.what());
	  		ros::Duration(1.0).sleep();
		}
		current_x = transform.getOrigin().x();
		current_y = transform.getOrigin().y();
		current_m = transform.getBasis();
		if(count == 0)
		{
			count = 1;
			last_x = current_x;
			last_y = current_y;
			last_m = current_m;
		}
		else if (count == 1)
		{
			velocity.linear.x = (current_x - last_x) / duration_time;
			velocity.linear.y = (current_y - last_y) / duration_time;

			//angular
			tfScalar current_yaw, current_pitch, current_roll, last_yaw, last_pitch, last_roll;
			current_m.getRPY(current_roll, current_pitch, current_yaw);
			last_m.getRPY(last_roll, last_pitch, last_yaw);
			velocity.angular.z = (current_roll - last_roll) / duration_time;

			ROS_INFO("Velocity: %f, %f, %f", velocity.linear.x, velocity.linear.y, velocity.angular.z);
			last_x = current_x;
			last_y = current_y;
			last_m = current_m;
			//publish velocity topic
			velocity_pub.publish(velocity);
		}	
		loop_rate.sleep();
	}
	return 0;
}
