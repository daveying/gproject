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
    ~UR5ControlInterface();
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
    
    bool joint_data_come;
    bool mouse_data_come;

    //subscribe the /joint_states from ur_modern_driver module
    ros::Subscriber sub_joint_states;
    //subscribe the TODO/mouse_speeds from mouse motion capture module
    ros::Subscriber sub_desired_velocities;
    //publish calculated joint velocities to ur_modern_driver module
    ros::Publisher pub_joint_velocities;
    //publish calculated tcp velocities for diagnose
    ros::Publisher pub_tcp_velocities;
}
