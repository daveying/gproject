/**
 * # Description
 *   Control the ur5 arm by specify tcp velocities respect to /ur_base_link frame
 *   
 * # Topic in
 *   /joint_states(sensor_msgs/JointState) joint sensor information from robot
 *   /desired_speeds(geometry_msgs/TwistStamped) control command from other nodes
 *
 * # Topic out
 *   /tcp_velocities(geometry_msgs/TwistStamped) tcp velocities respect to /ur_base_link for diagnose
 *   /ur_driver/joint_speed(trajectory_msgs/JointTrajectory) desired joint speed to robot
 *
 * # Broadcasted tf
 *   Last link frame(which is `wrist_3_link` in ur5 case) to reference frame. reference frame
 *   is the frame of reference_point_position
 *
 * # To run this node, first run
 * $ roscore
 * $ rosrun hokuyo_node hokuyo_node
 * $ roslaunch aimm_moveit_config aimm_moveit_planning_execution.launch robot_ip:=192.168.2.102
 * $ roslaunch laser_scan_matcher localization.launch
 *
 * # run this node
 * $ rosrun ur5_jacobian ur_control_interface
 */


#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TwistStamped.h>
#include <trajectory_msgs/JointTrajectory.h>
#include "tf/transform_broadcaster.h"

using namespace std;

const double pi = 3.1415926;

//========================global variables declare==============================//
//store recieved joint velocity values
Eigen::MatrixXd joint_velocities(6, 1);
//store recieved time stamp of joint velocity msg
std_msgs::Header header;

//store recieved joint angle values
vector<double> joint_values;

//store recieved desired motion data
Eigen::MatrixXd desired_velocities(6, 1);

bool joint_data_come = false;
bool desired_data_come = false;

//subscribe the /joint_states from ur_modern_driver module
ros::Subscriber sub_joint_states;
//subscribe the /desired_speeds from other nodes
ros::Subscriber sub_desired_speed;
//publish calculated joint speed to ur_modern_driver module
ros::Publisher pub_joint_speed;
//publish calculated tcp speed for diagnose
ros::Publisher pub_tcp_speed;

//===========================function declare====================================//

//callback function of ros subscriber sub_joint_states
//copy the joint angle values and joint velocity values 
//from msg to vector joint_values and joint_velocities
//and set flag joint_data_come to be true
void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);

//callback function of ros subscriber sub_desired_speed
//copy the desired speed from msg to vector desired_velocities
//and set flag desired_data_come to be true
void desiredSpeedsCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);

//===============================main logic======================================//

int main(int argc, char **argv)
{
    joint_values.resize(6, 0);
    ros::init(argc, argv, "ur5_control_interface");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    sub_joint_states = n.subscribe("/joint_states", 1000, jointStateCallback);
    sub_desired_speed = n.subscribe("/desired_speeds", 1000, desiredSpeedsCallback);
    
    pub_tcp_speed = n.advertise<geometry_msgs::TwistStamped>("/tcp_velocities", 1000);
    pub_joint_speed = n.advertise<trajectory_msgs::JointTrajectory>("/ur_driver/joint_speed", 1000);
    
    //get kinemetic model
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    ROS_INFO("Model frame(base_frame): %s", kinematic_model->getModelFrame().c_str());
    //create a robot_state object, we can set it to any configuration and then calculate the jacobian matrix coreesponding to this configuration
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
    const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("manipulator");
    const vector<string> &joint_names = joint_model_group->getJointModelNames();
    
    /* print out joint names
    for(int i = 0; i < joint_names.size(); i++)
    {
        ROS_INFO("%d joint_name: %s", i, joint_names[i].c_str());
    }*/
    
    //create matrixes for jacobian acquiring and joint speed calculation
    Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0); //respect to specified frame: last frame of `manipulator` group
    Eigen::MatrixXd jacobian; // for jacobian acquiring
    
    Eigen::MatrixXd joint_ctrl_velocities(6, 1);
    Eigen::MatrixXd tcp_velocities(6, 1);
    
    // current velocities of tcp calculated by current joint speeds.
    geometry_msgs::TwistStamped tcp_msg;
    //msg to ur_modern_driver
    trajectory_msgs::JointTrajectory trj;
    trajectory_msgs::JointTrajectoryPoint trjp;
    trjp.velocities.resize(6, 0);
    trj.points.push_back(trjp);
    
    //reference_point_position is respected to this link
    ROS_INFO("Last link name: %s\n", joint_model_group->getLinkModelNames().back().c_str()); 
    //broadcast transform from last link frame to reference frame
    tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion orien;
    orien.setRPY(pi / 2, -pi / 2, 0);
    transform.setRotation(orien);
    transform.setOrigin(tf::Vector3(reference_point_position(0), reference_point_position(1), reference_point_position(2)));
    
    while(ros::ok())
    {
        if(desired_data_come || joint_data_come)
        {
            desired_data_come = false; joint_data_come = false;
            
            //set robot configuration
            kinematic_state->setJointGroupPositions(joint_model_group, joint_values);
            //get jacobian of this configuration
            kinematic_state->getJacobian(joint_model_group, kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()), reference_point_position, jacobian);
            //ROS_INFO_STREAM("Jacobian: \n" << jacobian);
            //ROS_INFO("Last link name: %s\n", ((kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()))->getName()).c_str());
            
            
            // calculate tcp velocities for debug
            tcp_velocities = jacobian * joint_velocities;
            //ROS_INFO_STREAM("TCP velocities: \n" << tcp_velocities.transpose());
            // msg for debug
            tcp_msg.header = header;
            tcp_msg.twist.linear.x = tcp_velocities(0, 0);
            tcp_msg.twist.linear.y = tcp_velocities(1, 0);
            tcp_msg.twist.linear.z = tcp_velocities(2, 0);
            tcp_msg.twist.angular.x = tcp_velocities(3, 0);
            tcp_msg.twist.angular.y = tcp_velocities(4, 0);
            tcp_msg.twist.angular.z = tcp_velocities(5, 0);
            pub_tcp_speed.publish(tcp_msg);
            
            // calculate desired joint velocity according to desired tcp velocity
            joint_ctrl_velocities = jacobian.inverse() * desired_velocities;
            // fill the desired joint velocity to msg, which will be sent to ur_modern_driver module
            for(int i = 0; i < trj.points[0].velocities.size(); i++)
            {
                trj.points[0].velocities[i] = joint_ctrl_velocities(i, 0);
            }
            
        }
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), joint_model_group->getLinkModelNames().back(), "reference"));
        trj.header.stamp = ros::Time::now();
        pub_joint_speed.publish(trj);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    if(msg->velocity.size() == 6)
    {
        for(int i = 0; i < msg->velocity.size(); i++)
        {
            joint_velocities(i, 0) = msg->velocity[i];
            joint_values[i] = msg->position[i];
        }
        header = msg->header;
        joint_data_come = true;
    }
}

void desiredSpeedsCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    desired_velocities(0, 0) = msg->twist.linear.x;
    desired_velocities(1, 0) = msg->twist.linear.y;
    desired_velocities(2, 0) = msg->twist.linear.z;
    desired_velocities(3, 0) = msg->twist.angular.x;
    desired_velocities(4, 0) = msg->twist.angular.y;
    desired_velocities(5, 0) = msg->twist.angular.z;
    desired_data_come = true;
}














