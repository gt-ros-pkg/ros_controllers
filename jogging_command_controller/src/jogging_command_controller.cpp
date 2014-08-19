
// Pluginlib
#include <pluginlib/class_list_macros.h>

// Project
#include <jogging_command_controller/jogging_command_controller.h>

namespace position_controllers
{
  typedef jogging_command_controller::JoggingCommandController<
                                             hardware_interface::PositionJointInterface>
          JoggingCommandController;
}

namespace effort_controllers
{
  typedef jogging_command_controller::JoggingCommandController<
                                             hardware_interface::EffortJointInterface>
          JoggingCommandController;
}

PLUGINLIB_EXPORT_CLASS(position_controllers::JoggingCommandController, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(effort_controllers::JoggingCommandController,   controller_interface::ControllerBase)
