
// Pluginlib
#include <pluginlib/class_list_macros.h>

// Project
#include <jogging_command_controller/jogging_command_controller.h>
#include <joint_trajectory_controller/vel_fwd_hw_iface_adapter.h>

namespace vel_fwd_controllers
{
  typedef jogging_command_controller::JoggingCommandController<
                                               hardware_interface::VelocityJointInterface>
          JoggingCommandController;
}

PLUGINLIB_EXPORT_CLASS(vel_fwd_controllers::JoggingCommandController, 
                       controller_interface::ControllerBase)
