
// Pluginlib
#include <pluginlib/class_list_macros.h>

#include <joint_trajectory_controller/hardware_interface_adapter.h>
#include <trajectory_interface/pos_vel_acc_state.h>

// Project
#include <cartesian_position_controller/cartesian_position_controller.h>

namespace position_controllers
{
  typedef trajectory_interface::PosVelAccState<double> State;
  typedef PositionHardwareInterfaceAdapter<State> HwIfaceAdapter;
  typedef cartesian_position_controller::CartesianPositionController<State, HwIfaceAdapter>
          CartesianPositionController;
}

namespace effort_controllers
{
  typedef trajectory_interface::PosVelAccState<double> State;
  typedef EffortHardwareInterfaceAdapter<State> HwIfaceAdapter;
  typedef cartesian_position_controller::CartesianPositionController<State, HwIfaceAdapter>
          CartesianPositionController;
}

namespace vel_fwd_controllers
{
  typedef trajectory_interface::PosVelAccState<double> State;
  typedef VelocityForwardHardwareInterfaceAdapter<State> HwIfaceAdapter;
  typedef cartesian_position_controller::CartesianPositionController<State, HwIfaceAdapter>
          CartesianPositionController;
}

namespace vel_pid_controllers
{
  typedef trajectory_interface::PosVelAccState<double> State;
  typedef VelocityPIDHardwareInterfaceAdapter<State> HwIfaceAdapter;
  typedef cartesian_position_controller::CartesianPositionController<State, HwIfaceAdapter>
          CartesianPositionController;
}

PLUGINLIB_EXPORT_CLASS(position_controllers::CartesianPositionController, 
                       controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(effort_controllers::CartesianPositionController,   
                       controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(vel_fwd_controllers::CartesianPositionController, 
                       controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(vel_pid_controllers::CartesianPositionController, 
                       controller_interface::ControllerBase)
