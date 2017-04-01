#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TwistStamped.h>
#include <trajectory_msgs/JointTrajectory.h>
using namespace std;

class UR5ControlInterface
{
public:
    UR5ControlInterface(ros::NodeHandle &nh);
    ~UR5ControlInterface(){};//TODO 
    Eigen::MatrixXd& getJacobian(){return jacobian;};
private:
    //store recieved joint velocity values
    Eigen::MatrixXd joint_velocities;
    //store recieved time stamp of joint velocity msg
    std_msgs::Header header;
    
    //store recieved joint angle values
    vector<double> joint_values;
    
    //store recieved desired tcp velocity data, 
    //mouse_speeds[0] is x direction linear velocity
    Eigen::MatrixXd desired_velocities;
    
    Eigen::MatrixXd jacobian;
    Eigen::MatrixXd joint_ctrl_velocities;
    Eigen::MatrixXd tcp_velocities;
    
    bool joint_data_come;
    bool desired_data_come;
    
    ros::Rate loop_rate;

    //subscribe the /joint_states from ur_modern_driver module
    ros::Subscriber sub_joint_states;
    
    //publish calculated joint speed to ur_modern_driver module
    ros::Publisher pub_joint_speed;
    
    robot_model_loader::RobotModelLoader robot_model_loader;
    robot_model::RobotModelPtr kinematic_model;
    robot_state::RobotStatePtr kinematic_state;
    robot_state::JointModelGroup* joint_model_group;
    
    //respect to specified frame: last frame of `manipulator` group.
    Eigen::Vector3d reference_point_position;
    
    trajectory_msgs::JointTrajectory trj;
    trajectory_msgs::JointTrajectoryPoint trjp;
    
    
    //methods
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    
};

UR5ControlInterface::UR5ControlInterface(ros::NodeHandle &nh) :joint_velocities(6, 1), desired_velocities(6, 1), joint_ctrl_velocities(6, 1), tcp_velocities(6, 1), loop_rate(100), robot_model_loader("robot_description")/*FIXME if name doesn't match*/, reference_point_position(0.0, 0.0, 0.0)
{
    //ros::AsyncSpinner spinner(1);
    //spinner.start();
    joint_data_come = false;
    desired_data_come = false;
    joint_values.resize(6, 0);
    sub_joint_states = nh.subscribe("/joint_states", 1000, &UR5ControlInterface::jointStateCallback, this);//FIXME if topic name doesn't match
    pub_joint_speed = nh.advertise<trajectory_msgs::JointTrajectory>("/ur_driver/joint_speed", 1000);//FIXME if topic name doesn't match
    
    //get kinemetic model
    kinematic_model = robot_model_loader.getModel();
    ROS_INFO("Model frame(base_frame): %s", kinematic_model->getModelFrame().c_str());
    robot_state::RobotStatePtr temp_k_s(new robot_state::RobotState(kinematic_model));
    kinematic_state = temp_k_s;
    kinematic_state->setToDefaultValues();
    joint_model_group = kinematic_model->getJointModelGroup("manipulator");//FIXME if name doesn't match
    
    trjp.velocities.resize(6, 0);
    trj.points.push_back(trjp);
    
    for(int i = 0; i < desired_velocities.size(); i++)
    {
        desired_velocities(i, 0) = 0;
    }
    //ros::spin();
    
}

void UR5ControlInterface::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
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
        kinematic_state->setJointGroupPositions(joint_model_group, joint_values);
        kinematic_state->getJacobian(joint_model_group, kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()), reference_point_position, jacobian);
        //ROS_INFO_STREAM("Jacobian: \n" << jacobian);
        
        tcp_velocities = jacobian * joint_velocities;
        ROS_INFO_STREAM("TCP velocities: \n" << tcp_velocities.transpose());
    }
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ur_jacobian_control_interface");
    ros::NodeHandle nh;
    UR5ControlInterface ur5_interface(nh);
    Eigen::MatrixXd jacobian = ur5_interface.getJacobian();
    while(ros::ok())
    {
        ROS_INFO_STREAM("Jacobian: \n" << jacobian);
        ros::spinOnce();
    }
    ros::spin();
}










