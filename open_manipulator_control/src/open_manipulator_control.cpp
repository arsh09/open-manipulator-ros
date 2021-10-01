  
#include "open_manipulator_control/open_manipulator_control.h"
 
using namespace open_manipulator_control;

OpenManipulatorController::OpenManipulatorController(std::string usb_port, std::string baud_rate)
    :node_handle_(""),
    priv_node_handle_("~"),
{
  control_period_       = priv_node_handle_.param<double>("control_period", 0.010f);
  open_manipulator_.initOpenManipulator(using_platform_, usb_port, baud_rate, control_period_);
}

OpenManipulatorController::~OpenManipulatorController()
{

}

OpenManipulatorController::read()
{

}

OpenManipulatorController::write()
{

}
