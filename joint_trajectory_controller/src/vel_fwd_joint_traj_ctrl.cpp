
// Pluginlib
#include <pluginlib/class_list_macros.h>

// Project
#include <trajectory_interface/quintic_spline_segment.h>
#include <joint_trajectory_controller/joint_trajectory_controller.h>
#include <joint_trajectory_controller/vel_fwd_hw_iface_adapter.h>

namespace vel_fwd_controllers
{
  /**
   * \brief Joint trajectory controller that represents trajectory segments as <b>quintic splines</b> and sends
   * commands to a \b velocity interface.
   */
  typedef trajectory_interface::QuinticSplineSegment<double> SegmentImpl;
  typedef typename joint_trajectory_controller::JointTrajectorySegment<SegmentImpl>::State State;
  typedef HardwareInterfaceAdapter<hardware_interface::VelocityJointInterface, State> HwIfaceAdapter;
  typedef joint_trajectory_controller::JointTrajectoryController<SegmentImpl, HwIfaceAdapter> JointTrajectoryController;
}
PLUGINLIB_EXPORT_CLASS(vel_fwd_controllers::JointTrajectoryController, controller_interface::ControllerBase)
