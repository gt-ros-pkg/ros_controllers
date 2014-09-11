
/// \author Kelsey Hawkins

#ifndef JOINT_TRAJECTORY_CONTROLLER_VEL_FWD_HW_IFACE_ADAPTER_H
#define JOINT_TRAJECTORY_CONTROLLER_VEL_FWD_HW_IFACE_ADAPTER_H

#include <cassert>
#include <string>
#include <vector>

#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>

#include <ros/node_handle.h>
#include <ros/time.h>

#include <control_toolbox/pid.h>
#include <hardware_interface/joint_command_interface.h>

#include <realtime_tools/realtime_publisher.h>
#include <joint_trajectory_controller/hardware_interface_adapter.h>

/**
 * \brief Adapter for a position-controlled hardware interface. Forwards desired positions as commands.
 */
template <class State>
class HardwareInterfaceAdapter<hardware_interface::VelocityJointInterface, State>
{
public:
  typedef hardware_interface::VelocityJointInterface HwIface;
  HardwareInterfaceAdapter() : joint_handles_ptr_(0) {}

  bool init(std::vector<hardware_interface::JointHandle>& joint_handles, ros::NodeHandle& controller_nh)
  {
    // Store pointer to joint handles
    joint_handles_ptr_ = &joint_handles;

    return true;
  }

  void starting(const ros::Time& time) 
  {
    // Reset PIDs, zero velocity commands
    for (unsigned int i = 0; i < joint_handles_ptr_->size(); ++i)
    {
      (*joint_handles_ptr_)[i].setCommand(0.0);
    }
  }

  void stopping(const ros::Time& time) 
  {
    // Reset PIDs, zero velocity commands
    for (unsigned int i = 0; i < joint_handles_ptr_->size(); ++i)
    {
      (*joint_handles_ptr_)[i].setCommand(0.0);
    }
  }

  void updateCommand(const ros::Time&     /*time*/,
                     const ros::Duration& /*period*/,
                     const State&         desired_state,
                     const State&         /*state_error*/)
  {
    // Forward desired position to command
    const unsigned int n_joints = joint_handles_ptr_->size();
    for (unsigned int i = 0; i < n_joints; ++i) {(*joint_handles_ptr_)[i].setCommand(desired_state.velocity[i]);}
  }

private:
  std::vector<hardware_interface::JointHandle>* joint_handles_ptr_;
};

#endif
