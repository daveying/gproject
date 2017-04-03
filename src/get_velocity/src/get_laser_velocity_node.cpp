#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <vector>
using namespace std;

double v_x = 0;
vector<double> scans;
vector<double> last_scans;
ros::Time last_t;
bool s = false;


void imuCallback(const sensor_msgs::LaserScan::ConstPtr &msg);

int main(int argc, char **argv)
{
    scans.resize(3, 0);
    last_scans.resize(3, 0);
	ros::init(argc, argv, "getLaserVelocity");
	ros::NodeHandle n;
	ros::Rate loop_rate(100);
	
	ros::Subscriber sub_imu = n.subscribe("/scan", 1000, imuCallback);
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

void imuCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    if(s == false)
    {
        int count = msg->ranges.size();
        last_scans[0] = msg->ranges[count / 2 - 1];
        last_scans[1] = msg->ranges[count / 2];
        last_scans[2] = msg->ranges[count / 2 + 1];
        last_t = msg->header.stamp;
        s = true;
    }
    else
    {
        int count = msg->ranges.size();
        scans[0] = msg->ranges[count / 2 - 1];
        scans[1] = msg->ranges[count / 2];
        scans[2] = msg->ranges[count / 2 + 1];
        
        v_x = ((scans[0] + scans[1] + scans[2] -last_scans[0] - last_scans[1] - last_scans[2])/ 3.0)/ (msg->header.stamp.toSec() - last_t.toSec());
        
        last_scans = scans;
        last_t = msg->header.stamp;
    }

}
