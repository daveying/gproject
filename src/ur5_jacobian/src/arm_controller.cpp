/**
 * # Description
 *   Arm PID + feedforward controller.
 *   Input: Pose difference between /reference frame and /goal frame, moving velocity of goal point
 *   Output: tcp velocities respect to /world frame
 *   
 * # Topic in
 *   /path_vel(geometry_msgs::TwistStamped) moving velocity of goal point for feedforward control
 *   /tcp_velocities(geometry_msgs/TwistStamped) actual velocity of tcp respect to /ur_base_link.
 *    Used in D control
 *
 * # Topic out
 *   
 *
 *
 * # run this node
 * $ rosrun ur5_jacobian arm_controller
 */
 
#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "geometry_msgs/TwistStamped.h"
#include <Eigen/Dense>
#include "moving_average.h"
#include <string>
using namespace std;

Eigen::MatrixXd path_vel(6, 1);

tf::Vector3 tcp_linear_vel;
tf::Vector3 tcp_angular_vel;

tf::Vector3 platform_linear_vel;
tf::Vector3 platform_angular_vel;

void updateDiff(tf::TransformListener &tf_listener, Eigen::Vector3d &diff_vector, tf::Vector3 &axis, double &shortest_angle);
void movingAvarage(Eigen::MatrixXd &joint_ctrl_velocities, std::vector<double> &result);
void integralDiff(Eigen::Vector3d &diff_vector, double shortest_angle, Eigen::Vector3d &integral_vector, double &integral_angle);
double calcNorm(Eigen::MatrixXd vector);
double calcNorm(tf::Vector3 vector);
void updateTransform(tf::TransformListener &tf_listener, tf::StampedTransform &ub_w_transform, string parent, string child);
void pathVelCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
void tcpVelCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
void platformVelCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_controller");
    ros::NodeHandle n;
	ros::Rate loop_rate(100);
    
    ros::Subscriber sub_path_vel = n.subscribe("/path_vel", 1000, pathVelCallback);
    ros::Subscriber sub_tcp_vel = n.subscribe("/tcp_velocities", 1000, tcpVelCallback);
    ros::Subscriber sub_platform_vel = n.subscribe("/platform_speeds", 1000, platformVelCallback);
    //ros::Publisher pub_tcp_desired_vel = n.advertise<geometry_msgs::TwistStamped>("/tcp_desired_vel_arm", 1000);
    ros::Publisher pub_tcp_desired_vel = n.advertise<geometry_msgs::TwistStamped>("/desired_speeds", 1000); //target vel feedforward
    
    tf::TransformListener tf_listener;
    Eigen::Vector3d diff_vector(0.0, 0.0, 0.0); //position diff between goal and reference, respect to world frame
    Eigen::Vector3d integral_vector(0.0, 0.0, 0.0); //integral of diff vector
    
    tf::Vector3 axis(0.0, 0.0, 0.0);
    double shortest_angle = 0;
    double integral_angle = 0;
    tf::Vector3 t_w_wxyz;
    
    //double Kp_pose = 4.4 or 3.0, Ki_pose = 0.000, Kd_pose = -0.2 or 0.1, Kf_pose = 1, Kpf_pose = -1;
    double Kp_pose = 0, Ki_pose = 0.000, Kd_pose = -0.0, Kf_pose = 0, Kpf_pose = -0;
    double Kp_orien = 4.4, Ki_orien = 0, Kd_orien = -0.0, Kpf_orien = -1;
    
    updateDiff(tf_listener, diff_vector, axis, shortest_angle);
    integralDiff(diff_vector, shortest_angle, integral_vector, integral_angle);
    
    tf::Vector3 command_linear_vel;
    tf::Vector3 command_angular_vel;
    
    tf::StampedTransform ub_w_transform;// /ur_base_link respect to /world
    tf::StampedTransform ref_ub_transform; // /reference respect to /ur_base_link
    
    MovingAverage smoother(10);
    
    while(ros::ok())
    {
        updateTransform(tf_listener, ub_w_transform, "/world", "/ur_base_link");
        ub_w_transform.setOrigin(tf::Vector3(0,0,0));
        
        tf::Vector3 tcp_linear_vel_world = ub_w_transform * tcp_linear_vel;
        tf::Vector3 tcp_angular_vel_world = ub_w_transform * tcp_angular_vel;
        
        updateTransform(tf_listener, ref_ub_transform, "/ur_base_link", "/reference");
        tf::Vector3 tcp_in_urbase = ref_ub_transform.getOrigin();
        tf::Vector3 tcp_in_world = ub_w_transform * tcp_in_urbase; // just rotate operation
        tf::Vector3 tcp_vel_in_world = platform_linear_vel + tf::Vector3(-platform_angular_vel[2] * tcp_in_world[1], platform_angular_vel[2] * tcp_in_world[0], 0);
        ROS_INFO("tcp_vel_in_world: %lf, %lf, %lf", tcp_vel_in_world[0], tcp_vel_in_world[1], tcp_vel_in_world[2]);
        
        for(int i = 0; i < 3; i++)
        {
            command_linear_vel[i] = Kp_pose * diff_vector(i)              /*P*/
                                  + Ki_pose * integral_vector(i)          /*I*/
                                  + Kd_pose * tcp_linear_vel_world[i]     /*D*/
                                  + Kf_pose * path_vel(i, 0)              /*Feedforward*/
                                  + Kpf_pose * tcp_vel_in_world[i];       /*Feedforward of platform vel*/
        }
        double linear_v = calcNorm(command_linear_vel);
        double max_linear_speed = 0.15; //TODO, change the linear speed limit
        if(linear_v > max_linear_speed)
        {
            command_linear_vel = command_linear_vel * max_linear_speed / linear_v;
        }
        double angular_v = Kp_orien * shortest_angle
                            + Ki_orien * integral_angle
                            + Kd_orien * calcNorm(tcp_angular_vel_world);
                            + Kpf_orien * platform_angular_vel;
        double max_angular_speed = 10.4; //TODO, change the angular speed limit
        if(angular_v > max_angular_speed)
        {
            angular_v = max_angular_speed;
        }
        command_angular_vel = axis * angular_v; //axis is respect to world frame
        
        geometry_msgs::TwistStamped tcp_vel_msg;
        tcp_vel_msg.header.stamp = ros::Time::now();
        
        //====================TEST=============================//
        
        tf::Vector3 tcp_linear_vel_urbase = ub_w_transform.inverse() * command_linear_vel;
        tf::Vector3 tcp_angular_vel_urbase = ub_w_transform.inverse() * command_angular_vel;
        ROS_INFO("command: %f, %f, %f, %f, %f, %f", tcp_linear_vel_urbase[0], tcp_linear_vel_urbase[1], tcp_linear_vel_urbase[2], tcp_angular_vel_urbase[0], tcp_angular_vel_urbase[1], tcp_angular_vel_urbase[2]);
        
        Eigen::MatrixXd temp(6, 1);
        temp(0, 0) = tcp_linear_vel_urbase[0];
        temp(1, 0) = tcp_linear_vel_urbase[1];
        temp(2, 0) = tcp_linear_vel_urbase[2];
        temp(3, 0) = tcp_angular_vel_urbase[0];
        temp(4, 0) = tcp_angular_vel_urbase[1];
        temp(5, 0) = tcp_angular_vel_urbase[2];
        smoother.in(temp);
        Eigen::MatrixXd temp2 = smoother.out();
        
        
        
        tcp_vel_msg.twist.linear.x = temp(0, 0);
        tcp_vel_msg.twist.linear.y = temp(1, 0);
        tcp_vel_msg.twist.linear.z = temp(2, 0);
        tcp_vel_msg.twist.angular.x = temp(3, 0);
        tcp_vel_msg.twist.angular.y = temp(4, 0);
        tcp_vel_msg.twist.angular.z = temp(5, 0); 
        //====================TEST END==========================//
        /*
        tcp_vel_msg.twist.linear.x = command_linear_vel[0];
        tcp_vel_msg.twist.linear.y = command_linear_vel[1];
        tcp_vel_msg.twist.linear.z = command_linear_vel[2];
        tcp_vel_msg.twist.angular.x = command_angular_vel[0];
        tcp_vel_msg.twist.angular.y = command_angular_vel[1];
        tcp_vel_msg.twist.angular.z = command_angular_vel[2];*/

        pub_tcp_desired_vel.publish(tcp_vel_msg);
        
        updateDiff(tf_listener, diff_vector, axis, shortest_angle);
		integralDiff(diff_vector, shortest_angle, integral_vector, integral_angle);
		ros::spinOnce();
		loop_rate.sleep();
    }
    ros::spin();
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
    tcp_linear_vel[0] = msg->twist.linear.x;
    tcp_linear_vel[1] = msg->twist.linear.y;
    tcp_linear_vel[2] = msg->twist.linear.z;
    tcp_angular_vel[0] = msg->twist.angular.x;
    tcp_angular_vel[1] = msg->twist.angular.y;
    tcp_angular_vel[2] = msg->twist.angular.z;
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


double calcNorm(Eigen::MatrixXd vector)
{
	return sqrt(vector(0,0)*vector(0,0) + vector(1,0)*vector(1,0) + vector(2,0)*vector(2,0));
}

double calcNorm(tf::Vector3 vector)
{
	return sqrt(vector[0]*vector[0] + vector[1]*vector[1] + vector[2]*vector[2]);
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
			ros::Duration(1).sleep();
			continue;
        }
    }
}

void updateDiff(tf::TransformListener &tf_listener, Eigen::Vector3d &diff_vector, tf::Vector3 &axis, double &shortest_angle)
{	

	tf::StampedTransform g_w_transform;//goal respect to world
	tf::StampedTransform t_w_transform;//tool respect to world
	tf::Transform g_t_transform;//goal respect to tool
	tf::Quaternion g_t_rotation;

	bool s = false;
	while(!s && ros::ok())
	{
		try
		{// ur_base_link --> base_link, goal --> ar_grasp_e
			tf_listener.lookupTransform("/world", "/reference", ros::Time(0), t_w_transform);
			tf_listener.lookupTransform("/world", "/goal", ros::Time(0), g_w_transform);
			g_t_transform = t_w_transform.inverse() * g_w_transform;
			g_t_rotation = g_t_transform.getRotation();

			//third return value
			shortest_angle = g_t_rotation.getAngle();
			//ROS_INFO("angle: %lf", shortest_angle);
			if(shortest_angle < 0.01)shortest_angle = 0;

			tf::StampedTransform t_w_rotation = t_w_transform;
			t_w_rotation.setOrigin(tf::Vector3(0,0,0));

			//second return value, axis is respect to world frame
			axis = t_w_rotation * g_t_rotation.getAxis();

			//first return value
			diff_vector(0) = g_w_transform.getOrigin().x() - t_w_transform.getOrigin().x();
			diff_vector(1) = g_w_transform.getOrigin().y() - t_w_transform.getOrigin().y();
			diff_vector(2) = g_w_transform.getOrigin().z() - t_w_transform.getOrigin().z();
			//ROS_INFO("diff vector:%lf, %lf, %lf\n" , diff_vector(0), diff_vector(1), diff_vector(2));
			s = true;
		}
		catch (tf::TransformException &ex)
		{
			ROS_ERROR("%s", ex.what());
			ros::Duration(1).sleep();
			continue;
		}
	}
	
}

void integralDiff(Eigen::Vector3d &diff_vector, double shortest_angle, Eigen::Vector3d &integral_vector, double &integral_angle)
{
	//TODO, change the integral time
	int count = 150; //100Hz, 50 is represents 0.5s
	static std::vector<Eigen::Vector3d> vector_c(count, Eigen::Vector3d(0, 0, 0));
	static int index = 0;
	vector_c[index] = diff_vector;
	index++;
	if(index == count)index = 0;
	for(int i = 0; i != vector_c.size(); ++i)
		integral_vector += vector_c[i];

	static std::vector<double> angle_c(count, 0);
	static int index2 = 0;
	angle_c[index2] = shortest_angle;
	index2++;
	if(index2 == count)index2 = 0;
	for(int i = 0; i != angle_c.size(); ++i)
		integral_angle += angle_c[i];
}
