  
#include "open_manipulator_control/open_manipulator_hw_interface.h"
 

namespace open_manipulator_control
{

OpenManipulatorController::OpenManipulatorController(std::string usb_port, std::string baud_rate)
    :node_handle_(""),
    priv_node_handle_("~")
{
    
    control_period_       = priv_node_handle_.param<double>("control_period", 0.010f);
    open_manipulator_.initOpenManipulator(using_platform_, usb_port, baud_rate, control_period_);
    // open_manipulator_.initOpenManipulator(using_platform_, usb_port, baud_rate, control_period_);

    pos_.resize( total_joints );
    vel_.resize( total_joints );
    eff_.resize( total_joints );

    for (int i = 0; i < total_joints; i++ )
    {
        std::stringstream ss;
        if ( i != total_joints - 1)
        {
            ss << "joint" << i + 1;
        }
        else
        {
            ss << "gripper";
        }
        
        hardware_interface::JointStateHandle state_handle(ss.str(), &pos_[i], &vel_[i], &eff_[i]);
        jntStInterface_.registerHandle(state_handle);

        // Joint position handle for each joint in arm (+ gripper) - position controller
        hardware_interface::JointHandle pos_handle(jntStInterface_.getHandle(ss.str()), &cmd_[i]);
        posJntInterface_.registerHandle(pos_handle);
    }

    // register interfaces
    registerInterface(&jntStInterface_);
    registerInterface(&posJntInterface_);

}

OpenManipulatorController::~OpenManipulatorController()
{
    log::info("Shutdown the OpenManipulator");
    open_manipulator_.disableAllActuator();
    ros::shutdown();
}

bool OpenManipulatorController::read(ros::Time time, ros::Duration period)
{
    ROS_INFO("Read joint states and put it in pos_, vel_ eff_ variables");    

    // array of joint values where each value is struct of position, vel, acc, effort 
    std::vector<JointValue> all_joint_values = open_manipulator_.getAllJointValue();
    std::vector<JointValue> all_tool_values = open_manipulator_.getAllToolValue();

    int arm_joints = 0;
    for (size_t i = 0; i < all_joint_values.size(); i++)
    {
        pos_[i] = all_joint_values[i].position;
        vel_[i] = all_joint_values[i].velocity;
        eff_[i] = all_joint_values[i].effort;
        arm_joints += 1;
    }
    for (size_t j = arm_joints; j < all_tool_values.size(); j++)
    {
        pos_[j] = all_joint_values[j].position;
        vel_[j] = all_joint_values[j].velocity;
        eff_[j] = all_joint_values[j].effort;
    }

    return true;
}

bool OpenManipulatorController::write(ros::Time time, ros::Duration period)
{
    ROS_INFO("Write joint states here");

    std::vector<Name> arm_joint_names;
    std::vector<Name> gripper_joint_names;
    std::vector<JointValue> arm_joint_values;
    std::vector<JointValue> gripper_joint_values;

    arm_joint_names.resize(6);
    gripper_joint_names.resize(6);
    arm_joint_values.resize(1);
    gripper_joint_values.resize(1);

    for (int i = 0; i < cmd_.size(); i++)
    {
        JointValue joint;
        joint.position = cmd_[0];
        joint.velocity = 0;
        joint.effort = 0;

        if ( i != cmd_.size() - 1)
        {   
            // arm
            Name name = "joint" + std::to_string(i+1);
            arm_joint_names.push_back(name);
            arm_joint_values.push_back(joint);
        }
        else
        {
            // gripper
            Name name = "gripper";
            gripper_joint_names.push_back(name);
            gripper_joint_values.push_back(joint);
        }
    }

    open_manipulator_.sendMultipleJointActuatorValue(arm_joint_names, arm_joint_values);
    open_manipulator_.sendMultipleToolActuatorValue(gripper_joint_names, gripper_joint_values);

    return true;
}

}
