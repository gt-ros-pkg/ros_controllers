
// Pluginlib
#include <pluginlib/class_list_macros.h>

#include <joint_trajectory_controller/hardware_interface_adapter.h>
#include <trajectory_interface/pos_vel_acc_state.h>

// Project
#include <jogging_command_controller/jogging_command_controller.h>

namespace position_controllers
{
  typedef trajectory_interface::PosVelAccState<double> State;
  typedef PositionHardwareInterfaceAdapter<State> HwIfaceAdapter;
  typedef jogging_command_controller::JoggingCommandController<State, HwIfaceAdapter>
          JoggingCommandController;
}

namespace effort_controllers
{
  typedef trajectory_interface::PosVelAccState<double> State;
  typedef EffortHardwareInterfaceAdapter<State> HwIfaceAdapter;
  typedef jogging_command_controller::JoggingCommandController<State, HwIfaceAdapter>
          JoggingCommandController;
}

namespace vel_fwd_controllers
{
  typedef trajectory_interface::PosVelAccState<double> State;
  typedef VelocityForwardHardwareInterfaceAdapter<State> HwIfaceAdapter;
  typedef jogging_command_controller::JoggingCommandController<State, HwIfaceAdapter>
          JoggingCommandController;
}

namespace vel_pid_controllers
{
  typedef trajectory_interface::PosVelAccState<double> State;
  typedef VelocityPIDHardwareInterfaceAdapter<State> HwIfaceAdapter;
  typedef jogging_command_controller::JoggingCommandController<State, HwIfaceAdapter>
          JoggingCommandController;
}

PLUGINLIB_EXPORT_CLASS(position_controllers::JoggingCommandController, 
                       controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(effort_controllers::JoggingCommandController,   
                       controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(vel_fwd_controllers::JoggingCommandController, 
                       controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(vel_pid_controllers::JoggingCommandController, 
                       controller_interface::ControllerBase)
