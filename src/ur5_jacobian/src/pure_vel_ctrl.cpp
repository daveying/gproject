#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TwistStamped.h>
#include <trajectory_msgs/JointTrajectory.h>

using namespace std;

//========================global variables declare==============================//
//store recieved joint velocity values
vector<double> joint_speeds;
//store recieved time stamp of joint velocity msg
std_msgs::Header header;

//store recieved joint angle values
vector<double> joint_values;

//store recieved mouse motion data, 
//mouse_speeds[0] is x direction velocity
vector<double> mouse_speeds;

bool joint_data_come = false;
bool mouse_data_come = false;

//subscribe the /joint_states from ur_modern_driver module
ros::Subscriber sub_joint_states;
//subscribe the /mouse_speeds from mouse motion capture module
ros::Subscriber sub_platform_speed;
//publish calculated joint speed to ur_modern_driver module
ros::Publisher pub_joint_speed;
//publish calculated tcp speed for diagnose
ros::Publisher pub_tcp_speed;

//===========================function declare====================================//

//fill a vector with zeros. the number of zeros is defined by paramenter `length`.
void fillZeros(vector<double> &vec, int length);

//init the container vectors, very specific function!!
void initContainers();

//copy values from vector src to vector dist, number of values is defined by paramenter `length`
void copyValues(const vector<double> &src, vector<double> &dist, int length);

//callback function of ros subscriber sub_joint_states
//copy the joint angle values and joint velocity values 
//from msg to vector joint_values and joint_speeds
//and set flag joint_data_come to be true
void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);

//callback function of ros subscriber sub_platform_speed
//copy the mouse speed from msg to vector mouse_speeds
//and set flag mouse_data_come to be true
void mouseSpeedsCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);

//===============================main logic======================================//

int main(int argc, char **argv)
{
    initContainers();
    ros::init(argc, argv, "pure_vel_ctrl");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    sub_joint_states = n.subscribe("/joint_states", 1000, jointStateCallback);
    sub_platform_speed = n.subscribe("/mouse_speeds", 1000, mouseSpeedsCallback);
    
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
    Eigen::MatrixXd joint_velocities(6, 1);
    Eigen::MatrixXd joint_ctrl_velocities(6, 1);
    Eigen::MatrixXd tcp_velocities(6, 1);
    Eigen::MatrixXd platform_velocities(6, 1);
    
    //TODO: figure out what is this
    geometry_msgs::TwistStamped tcp_msg;
    //msg to ur_modern_driver
    trajectory_msgs::JointTrajectory trj;
    trajectory_msgs::JointTrajectoryPoint trjp;
    fillZeros(trjp.velocities, 6);
    trj.points.push_back(trjp);
    
    while(ros::ok())
    {
        if(mouse_data_come || joint_data_come)
        {
            mouse_data_come = false; joint_data_come = false;
            
            //set robot configuration
            kinematic_state->setJointGroupPositions(joint_model_group, joint_values);
            //get jacobian of this configuration
            kinematic_state->getJacobian(joint_model_group, kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()), reference_point_position, jacobian);
            ROS_INFO_STREAM("Jacobian: \n" << jacobian);
            
            for (int i = 0; i < joint_speeds.size(); i++)
            {
                joint_velocities(i, 0) = joint_speeds[i];
            }
            
            // calculate tcp velocities for debug
            tcp_velocities = jacobian * joint_velocities;
            ROS_INFO_STREAM("TCP velocities: \n" << tcp_velocities);
            // msg for debug
            tcp_msg.header = header;
            tcp_msg.twist.linear.x = tcp_velocities(0, 0);
            tcp_msg.twist.linear.y = tcp_velocities(1, 0);
            tcp_msg.twist.linear.z = tcp_velocities(2, 0);
            tcp_msg.twist.angular.x = tcp_velocities(3, 0);
            tcp_msg.twist.angular.y = tcp_velocities(4, 0);
            tcp_msg.twist.angular.z = tcp_velocities(5, 0);
            pub_tcp_speed.publish(tcp_msg);
            
            double k = 1;
            platform_velocities(0, 0) = k * mouse_speeds[0];
            platform_velocities(1, 0) = k * mouse_speeds[1];
            platform_velocities(2, 0) = 0;
            platform_velocities(3, 0) = 0;
            platform_velocities(4, 0) = 0;
            platform_velocities(5, 0) = 0;
            
            //TODO: calculate tcp compensate velocities according to platform velocities
            // calculate desired joint velocity according to tcp compensate velocity
            joint_ctrl_velocities = jacobian.inverse() * platform_velocities;
            // fill the desired joint velocity to msg, which will be sent to ur_modern_driver module
			trj.points[0].velocities[0] = joint_ctrl_velocities(0,0);
			trj.points[0].velocities[1] = joint_ctrl_velocities(1,0);
			trj.points[0].velocities[2] = joint_ctrl_velocities(2,0);
			trj.points[0].velocities[3] = joint_ctrl_velocities(3,0);
			trj.points[0].velocities[4] = joint_ctrl_velocities(4,0);
			trj.points[0].velocities[5] = joint_ctrl_velocities(5,0);
            
        }
        trj.header.stamp = ros::Time::now();
        pub_joint_speed.publish(trj);
        ros::spinOnce();
        loop_rate.sleep();
    }
}



void initContainers()
{
    fillZeros(joint_speeds, 6);
    fillZeros(joint_values, 6);
    fillZeros(mouse_speeds, 3);
}

void fillZeros(vector<double> &vec, int length)
{
    for(int i = 0; i < length; i++)
    {
        vec.push_back(0.0);
    }
}

void copyValues(const vector<double> &src, vector<double> &dist, int length)
{
    for(int i = 0; i < length; i++)
    {
        dist[i] = src[i];
    }
}

void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    if(msg->velocity.size() == 6)
    {
        copyValues(msg->velocity, joint_speeds, 6);
        copyValues(msg->position, joint_values, 6);
        header = msg->header;
        joint_data_come = true;
    }
}

void mouseSpeedsCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    mouse_speeds[0] = msg->twist.linear.x;
    mouse_speeds[1] = msg->twist.linear.y;
    mouse_speeds[2] = 0;
    mouse_data_come = true;
}














