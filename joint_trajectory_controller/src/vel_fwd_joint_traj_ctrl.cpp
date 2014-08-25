
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
  typedef joint_trajectory_controller::JointTrajectoryController<trajectory_interface::QuinticSplineSegment<double>,
                                                                 hardware_interface::VelocityJointInterface>
          JointTrajectoryController;
}
PLUGINLIB_EXPORT_CLASS(vel_fwd_controllers::JointTrajectoryController, controller_interface::ControllerBase)
