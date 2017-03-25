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
void copyValues(vector<double> &src, vector<double> &dist, int length);

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
    
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
    const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("manipulator");
    const vector<string> &joint_names = joint_model_group->getJointModelNames();
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
        vec.push_back(0);
    }
}

void copyValues(vector<double> &src, vector<double> &dist, int length)
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
        joint_data_come = true;
    }
}

void mouseSpeedsCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    mouse_speeds[0] = msg->twist.angular.x;
    mouse_speeds[1] = msg->twist.angular.y;
    mouse_speeds[2] = 0;
    mouse_data_come = true;
}














