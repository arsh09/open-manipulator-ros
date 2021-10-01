#ifndef _OPEN_MANOPULATOR_CONTROL_
#define _OPEN_MANOPULATOR_CONTROL_

// C++ standard
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <cerrno>

// ROS
#include <boost/scoped_ptr.hpp>

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <hardware_interface/controller_info.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>


#include "open_manipulator_6dof_libs/open_manipulator.h"

 
namespace open_manipulator_control 
{
    class OpenManipulatorController: public hardware_interface::RobotHW
    {
        public:
            OpenManipulatorController(std::string usb_port, std::string baud_rate);
            ~OpenManipulatorController();

            read();
            write();

            // ROS NodeHandle
            ros::NodeHandle node_handle_;
            ros::NodeHandle priv_node_handle_;

            // ROS Parameters
            bool using_platform_;
            bool using_moveit_;
            double control_period_;
            
        private : 
    
    }

}