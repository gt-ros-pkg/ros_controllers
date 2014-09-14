///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2013, PAL Robotics S.L.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of PAL Robotics S.L. nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

/// \author Adolfo Rodriguez Tsouroukdissian

#ifndef JOINT_TRAJECTORY_CONTROLLER_HARDWARE_INTERFACE_ADAPTER_H
#define JOINT_TRAJECTORY_CONTROLLER_HARDWARE_INTERFACE_ADAPTER_H

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

#include <std_msgs/Float64MultiArray.h>

/**
 * \brief Helper class to simplify integrating the JointTrajectoryController with different hardware interfaces.
 *
 * The JointTrajectoryController outputs position, velocity, command triplets, while the more common hardware interfaces
 * it is supposed to work with accept either position or effort commands.
 *
 * Use one of the avaialble template specializations of this class (or create your own) to adapt the
 * JointTrajectoryController to a specidfic hardware interface.
 */
template <class State>
class HardwareInterfaceAdapterBase
{
public:

  virtual void starting(const ros::Time& time) = 0;
  virtual void stopping(const ros::Time& time) = 0;

  virtual void updateCommand(const ros::Time&     time,
                             const ros::Duration& period,
                             const State&         desired_state,
                             const State&         state_error) = 0;
};

template <class HardwareInterface, class State>
class HardwareInterfaceAdapter : public HardwareInterfaceAdapterBase<State>
{
public:
  typedef HardwareInterface HwIface;
  typedef typename HwIface::ResourceHandleType HandleType;
  virtual bool init(std::vector<HandleType>& joint_handles, ros::NodeHandle& controller_nh)
  {
    return false;
  }

  virtual void starting(const ros::Time& time) {}
  virtual void stopping(const ros::Time& time) {}

  virtual void updateCommand(const ros::Time&     time,
                             const ros::Duration& period,
                             const State&         desired_state,
                             const State&         state_error) {}
};

/**
 * \brief Adapter for a position-controlled hardware interface. Forwards desired positions as commands.
 */
template <class State>
class PositionHardwareInterfaceAdapter 
  : public HardwareInterfaceAdapter<hardware_interface::PositionJointInterface, State>
{
public:
  PositionHardwareInterfaceAdapter() : joint_handles_ptr_(0) {}

  bool init(std::vector<hardware_interface::JointHandle>& joint_handles, ros::NodeHandle& controller_nh)
  {
    // Store pointer to joint handles
    joint_handles_ptr_ = &joint_handles;

    return true;
  }

  void updateCommand(const ros::Time&     /*time*/,
                     const ros::Duration& /*period*/,
                     const State&         desired_state,
                     const State&         /*state_error*/)
  {
    // Forward desired position to command
    const unsigned int n_joints = joint_handles_ptr_->size();
    for (unsigned int i = 0; i < n_joints; ++i) {(*joint_handles_ptr_)[i].setCommand(desired_state.position[i]);}
  }

private:
  std::vector<hardware_interface::JointHandle>* joint_handles_ptr_;
};

/**
 * \brief Adapter for an effort-controlled hardware interface. Maps position and velocity errors to effort commands
 * through a position PID loop.
 *
 * The following is an example configuration of a controller that uses this adapter. Notice the \p gains entry:
 * \code
 * head_controller:
 *   type: "position_controllers/JointTrajectoryController"
 *   joints:
 *     - head_1_joint
 *     - head_2_joint
 *
 *   constraints:
 *     goal_time: 0.6
 *     stopped_velocity_tolerance: 0.02
 *     head_1_joint:
 *       trajectory: 0.05
 *       goal: 0.02
 *     head_2_joint:
 *       goal: 0.01
 *
 *   gains:
 *     head_1_joint: {p: 200, d: 1, i: 5, i_clamp: 1}
 *     head_2_joint: {p: 200, d: 1, i: 5, i_clamp: 1}
 * \endcode
 */
template <class State>
class EffortHardwareInterfaceAdapter 
  : public HardwareInterfaceAdapter<hardware_interface::EffortJointInterface, State>
{
public:
  EffortHardwareInterfaceAdapter() : joint_handles_ptr_(0) {}

  bool init(std::vector<hardware_interface::JointHandle>& joint_handles, ros::NodeHandle& controller_nh)
  {
    // Store pointer to joint handles
    joint_handles_ptr_ = &joint_handles;

    // Initialize PIDs
    pids_.resize(joint_handles.size());
    for (unsigned int i = 0; i < pids_.size(); ++i)
    {
      // Node handle to PID gains
      ros::NodeHandle joint_nh(controller_nh, std::string("gains/") + joint_handles[i].getName());

      // Init PID gains from ROS parameter server
      pids_[i].reset(new control_toolbox::Pid());
      if (!pids_[i]->init(joint_nh))
      {
        ROS_WARN_STREAM("Failed to initialize PID gains from ROS parameter server.");
        return false;
      }
    }

    return true;
  }

  void starting(const ros::Time& time)
  {
    if (!joint_handles_ptr_) {return;}

    // Reset PIDs, zero effort commands
    for (unsigned int i = 0; i < pids_.size(); ++i)
    {
      pids_[i]->reset();
      (*joint_handles_ptr_)[i].setCommand(0.0);
    }
  }

  void updateCommand(const ros::Time&     /*time*/,
                     const ros::Duration& period,
                     const State&         /*desired_state*/,
                     const State&         state_error)
  {
    const unsigned int n_joints = joint_handles_ptr_->size();

    // Preconditions
    if (!joint_handles_ptr_) {return;}
    assert(n_joints == state_error.position.size());
    assert(n_joints == state_error.velocity.size());

    // Update PIDs
    for (unsigned int i = 0; i < n_joints; ++i)
    {
      const double command = pids_[i]->computeCommand(state_error.position[i], state_error.velocity[i], period);
      (*joint_handles_ptr_)[i].setCommand(command);
    }
  }

private:
  typedef boost::shared_ptr<control_toolbox::Pid> PidPtr;
  std::vector<PidPtr> pids_;

  std::vector<hardware_interface::JointHandle>* joint_handles_ptr_;
};

/**
 * \brief Adapter for a position-controlled hardware interface. Forwards desired positions as commands.
 */
template <class State>
class VelocityForwardHardwareInterfaceAdapter 
  : public HardwareInterfaceAdapter<hardware_interface::VelocityJointInterface, State>
{
public:
  VelocityForwardHardwareInterfaceAdapter() : joint_handles_ptr_(0) {}

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

/**
 * \brief Adapter for an velocity-controlled hardware interface. Maps position and velocity errors to velocity commands
 * through a position PID loop.
 *
 * The following is an example configuration of a controller that uses this adapter. Notice the \p gains entry:
 * \code
 * head_controller:
 *   type: "velocity_controllers/JointTrajectoryController"
 *   joints:
 *     - head_1_joint
 *     - head_2_joint
 *
 *   constraints:
 *     goal_time: 0.6
 *     stopped_velocity_tolerance: 0.02
 *     head_1_joint:
 *       trajectory: 0.05
 *       goal: 0.02
 *     head_2_joint:
 *       goal: 0.01
 *
 *   gains:
 *     head_1_joint: {p: 200, d: 1, i: 5, i_clamp: 1}
 *     head_2_joint: {p: 200, d: 1, i: 5, i_clamp: 1}
 * \endcode
 */
template <class State>
class VelocityPIDHardwareInterfaceAdapter 
  : public HardwareInterfaceAdapter<hardware_interface::VelocityJointInterface, State>
{
public:
  VelocityPIDHardwareInterfaceAdapter() : joint_handles_ptr_(0) {}

  bool init(std::vector<hardware_interface::JointHandle>& joint_handles, ros::NodeHandle& controller_nh)
  {
    // Store pointer to joint handles
    joint_handles_ptr_ = &joint_handles;

    double cmd_publish_rate;
    controller_nh.param<double>("cmd_publish_rate", cmd_publish_rate, 100.0);
    cmd_pub_period_ = ros::Duration(1.0 / cmd_publish_rate);

    // Initialize PIDs
    pids_.resize(joint_handles.size());
    for (unsigned int i = 0; i < pids_.size(); ++i)
    {
      // Node handle to PID gains
      ros::NodeHandle joint_nh(controller_nh, std::string("gains/") + joint_handles[i].getName());

      // Init PID gains from ROS parameter server
      pids_[i].reset(new control_toolbox::Pid());
      if (!pids_[i]->init(joint_nh))
      {
        ROS_WARN_STREAM("Failed to initialize PID gains from ROS parameter server.");
        return false;
      }
    }

    command_publisher_.reset(new CommandPublisher(controller_nh, "ctrl_command", 1));
    {
      command_publisher_->lock();
      const unsigned int n_joints = joint_handles_ptr_->size();
      for (unsigned int i = 0; i < n_joints; ++i) 
        command_publisher_->msg_.data.push_back((*joint_handles_ptr_)[i].getCommand());
      command_publisher_->msg_.layout.dim.resize(1);
      command_publisher_->msg_.layout.dim[0].size = n_joints;
      command_publisher_->unlock();
    }

    return true;
  }

  void starting(const ros::Time& time)
  {
    if (!joint_handles_ptr_) {return;}

    // Reset PIDs, zero velocity commands
    for (unsigned int i = 0; i < pids_.size(); ++i)
    {
      pids_[i]->reset();
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

  void updateCommand(const ros::Time&     time,
                     const ros::Duration& period,
                     const State&         /*desired_state*/,
                     const State&         state_error)
  {
    const unsigned int n_joints = joint_handles_ptr_->size();

    // Preconditions
    if (!joint_handles_ptr_) {return;}
    assert(n_joints == state_error.position.size());
    assert(n_joints == state_error.velocity.size());

    // Update commands from PIDs
    for (unsigned int i = 0; i < n_joints; ++i)
    {
      const double vel_command = pids_[i]->computeCommand(state_error.position[i], 
                                                          state_error.velocity[i], 
                                                          period);
      (*joint_handles_ptr_)[i].setCommand(vel_command);
    }

    if (!cmd_pub_period_.isZero() && last_cmd_pub_time_ + cmd_pub_period_ < time)
    {
      if (command_publisher_ && command_publisher_->trylock())
      {
        last_cmd_pub_time_ += cmd_pub_period_;
        for (unsigned int i = 0; i < n_joints; ++i)
          command_publisher_->msg_.data[i] = (*joint_handles_ptr_)[i].getCommand();
        command_publisher_->unlockAndPublish();
      }
    }
  }

private:
  typedef boost::shared_ptr<control_toolbox::Pid> PidPtr;
  std::vector<PidPtr> pids_;

  std::vector<hardware_interface::JointHandle>* joint_handles_ptr_;

  typedef realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray> CommandPublisher;
  typedef boost::scoped_ptr<CommandPublisher> CommandPublisherPtr;
  CommandPublisherPtr command_publisher_;
  ros::Time last_cmd_pub_time_;
  ros::Duration cmd_pub_period_;
};

#endif // header guard
