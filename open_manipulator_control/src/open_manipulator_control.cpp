  
#include "open_manipulator_control/open_manipulator_control.h"
 

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
}

bool OpenManipulatorController::write(ros::Time time, ros::Duration period)
{
    ROS_INFO("Write joint states here");
}

}
