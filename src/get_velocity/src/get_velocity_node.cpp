#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"

double acc_x = 0;
ros::Time last;
bool s = false;
double v_x = 0;

void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "getImuVelocity");
	ros::NodeHandle n;
	ros::Rate loop_rate(100);
	
	ros::Subscriber sub_imu = n.subscribe("/imu/data", 1000, imuCallback);
	ros::Publisher pub_vel = n.advertise<geometry_msgs::TwistStamped>("/imu_vel", 1000);
	
	geometry_msgs::TwistStamped msg;
	while(ros::ok())
	{
	    msg.header.stamp = ros::Time::now();
	    msg.twist.linear.x = v_x;
	    pub_vel.publish(msg);
	    ros::spinOnce();
	    loop_rate.sleep();
	}
	ros::spin();
	

}

int count = 0;

void imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    if(s == false)
    {
        acc_x = (msg->linear_acceleration.x) * 9.8 / 0.0245;
        s = true;
        last = msg->header.stamp;
    }
    else
    {
        v_x += ((msg->header.stamp.toSec() - last.toSec()) * acc_x);
        acc_x = (msg->linear_acceleration.x) * 9.8 / 0.0245;
        last = msg->header.stamp;
        if(acc_x < 0.0007*9.8/0.0245 && acc_x > -0.0007*9.8/0.0245)
        {
            count++;
            if(count > 100)
            {
                v_x = 0;
                count = 101;
            }
        }
        else
            count = 0;
            
        ROS_INFO("%d", count);
        
    }
}
