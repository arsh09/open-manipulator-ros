
#include <controller_manager/controller_manager.h>
#include "open_manipulator_control/open_manipulator_control.h"
#include <thread>
#include <chrono>
#include <string.h>
#include <ros/callback_queue.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "open_manipulator_control_node");
    ros::NodeHandle nh;

    open_manipulator_control::OpenManipulatorController open_manipulator_control;

    ros::CallbackQueue queue;
    controller_manager::ControllerManager cm(&open_manipulator_control, nh);

    ros::Rate rate(100);
    ros::AsyncSpinner spinner(1); 
    spinner.start();
    ros::Time now = ros::Time::now();

    while (ros::ok())
    {
        ros::Duration dt = ros::Time::now() - now;
        ros::Time now = ros::Time::now();
        success = open_manipulator_control.read(ros::Time::now(), dt);
        if (!success){
            break;
        }
        
        cm.update(ros::Time::now(), dt); 

        success = open_manipulator_control.write(ros::Time::now(), dt);
        if (!success){
            break;
        }
        rate.sleep();
    }
    spinner.stop();
    ROS_INFO("Open manipulator controller has stopped.");
    return 0;
}

