#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "draw_circle");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);
    
    ros::Publisher pub_vel = n.advertise<geometry_msgs::TwistStamped>("/mouse_speeds", 1000);
    
    ros::Time begin_t = ros::Time::now();
    double v_ampli = 0.04; //unit m/s. amplitude of velocity
    double radius = 0.1; //unit m. radius of circle
    double w = v_ampli / radius; // angular velocity of circle
    ros::Duration t;
    geometry_msgs::TwistStamped vel_msg;
    while(ros::ok())
    {
        t = ros::Time::now() - begin_t;
        double dt = t.toSec();
        double vx = v_ampli * sin(w * dt);
        double vy = v_ampli * cos(w * dt);
    	vel_msg.header.stamp = ros::Time::now();
		vel_msg.twist.linear.x = vx;
		vel_msg.twist.linear.y = vy;
		vel_msg.twist.linear.z = 0;
		vel_msg.twist.angular.x = 0;
		vel_msg.twist.angular.y = 0;
		vel_msg.twist.angular.z = 0;
		pub_vel.publish(vel_msg);
		ros::spinOnce();
		loop_rate.sleep();
    }
}
