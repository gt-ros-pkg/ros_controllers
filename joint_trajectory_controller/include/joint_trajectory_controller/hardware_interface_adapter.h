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
#include <hardware_interface/pos_vel_acc_joint_interface.h>

#include <realtime_tools/realtime_publisher.h>

#include <std_msgs/Float64MultiArray.h>
namespace hardware_interface_adapter
{
namespace internal
{

std::vector<std::string> getStrings(const ros::NodeHandle& nh, const std::string& param_name)
{
  using namespace XmlRpc;
  XmlRpcValue xml_array;
  if (!nh.getParam(param_name, xml_array))
  {
    ROS_ERROR_STREAM("Could not find '" << param_name << "' parameter (namespace: " << nh.getNamespace() << ").");
    return std::vector<std::string>();
  }
  if (xml_array.getType() != XmlRpcValue::TypeArray)
  {
    ROS_ERROR_STREAM("The '" << param_name << "' parameter is not an array (namespace: " <<
                     nh.getNamespace() << ").");
    return std::vector<std::string>();
  }

  std::vector<std::string> out;
  for (int i = 0; i < xml_array.size(); ++i)
  {
    if (xml_array[i].getType() != XmlRpcValue::TypeString)
    {
      ROS_ERROR_STREAM("The '" << param_name << "' parameter contains a non-string element (namespace: " <<
                       nh.getNamespace() << ").");
      return std::vector<std::string>();
    }
    out.push_back(static_cast<std::string>(xml_array[i]));
  }
  return out;
}

}
}

/**
 * \brief Helper class to simplify integrating the JointTrajectoryController with different hardware interfaces.
 *
 * The JointTrajectoryController outputs position, velocity and acceleration command triplets, while the more common hardware
 * interfaces accept position, velocity or effort commands.
 *
 * Use one of the available template specializations of this class (or create your own) to adapt the
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

  std::vector<HandleType> joint_handles;

  virtual bool initJoints(std::string ctrl_name, std::vector<std::string>& joint_names, 
                          HwIface *hw, ros::NodeHandle& controller_nh, 
                          std::vector<hardware_interface::JointStateHandle>& joint_states)
  {
    const unsigned int n_joints = joint_names.size();
    
    joint_handles.resize(n_joints);
    joint_states.resize(n_joints);
    for (unsigned int i = 0; i < n_joints; ++i) {
      // Joint handle
      try {
        joint_handles[i] = hw->getHandle(joint_names[i]);
        joint_states[i] = static_cast<hardware_interface::JointStateHandle>(joint_handles[i]);
      }
      catch (...) {
        ROS_ERROR_STREAM_NAMED(ctrl_name, 
            "Could not find joint '" << joint_names[i] << "' in '" << 
            hardware_interface::internal::demangledTypeName<HwIface>() << "'.");
        return false;
      }
    }
    // Hardware interface adapter
    init(controller_nh);

    ROS_DEBUG_STREAM_NAMED(ctrl_name, 
        "Initialized adapter for controller '" << ctrl_name << "' with:" <<
        "\n- Number of joints: " << n_joints <<
        "\n- Hardware interface type: '" << 
        hardware_interface::internal::demangledTypeName<HwIface>() << "'");

    return true;
  }

  virtual bool init(ros::NodeHandle& controller_nh) = 0;

  virtual void starting(const ros::Time& time) {}
  virtual void stopping(const ros::Time& time) {}

  virtual void updateCommand(const ros::Time&     time,
                             const ros::Duration& period,
                             const State&         desired_state,
                             const State&         state_error) {}
};

template <class HardwareInterface1, class HardwareInterface2, class State>
class HardwareInterfaceAdapter2 : public HardwareInterfaceAdapterBase<State>
{
public:
  typedef HardwareInterface1 HwIface1;
  typedef HardwareInterface2 HwIface2;
  typedef typename HwIface1::ResourceHandleType HandleType1;
  typedef typename HwIface2::ResourceHandleType HandleType2;

  virtual bool initJoints(std::string ctrl_name, std::vector<std::string>& joint_names, 
                          HwIface1 *hw1, HwIface2 *hw2, ros::NodeHandle& controller_nh, 
                          std::vector<hardware_interface::JointStateHandle>& joint_states) = 0;

  virtual void starting(const ros::Time& time) {}
  virtual void stopping(const ros::Time& time) {}

  virtual void updateCommand(const ros::Time&     time,
                             const ros::Duration& period,
                             const State&         desired_state,
                             const State&         state_error) {}
};

template <class HwIfaceAdapter1, class HwIfaceAdapter2, class State>
class NaiveHardwareInterfaceAdapter2 : 
  public HardwareInterfaceAdapter2<typename HwIfaceAdapter1::HwIface, 
                                   typename HwIfaceAdapter2::HwIface, State>
{
public:
  typedef typename HwIfaceAdapter1::HwIface HwIface1;
  typedef typename HwIfaceAdapter2::HwIface HwIface2;
  typedef typename HwIface1::ResourceHandleType HandleType1;
  typedef typename HwIface2::ResourceHandleType HandleType2;

  HwIfaceAdapter1 adapter1;
  HwIfaceAdapter2 adapter2;
  State desired_state1;
  State desired_state2;
  State state_error1;
  State state_error2;
  std::vector<int> adapter_inds;
  std::vector<std::string> adapter_namespaces;
  std::vector<std::string> joint_adapters;

  virtual bool initJoints(std::string ctrl_name, std::vector<std::string>& joint_names, 
                          HwIface1 *hw1, HwIface2 *hw2, ros::NodeHandle& controller_nh, 
                          std::vector<hardware_interface::JointStateHandle>& joint_states)
  {
    const unsigned int n_joints = joint_names.size();

    std::vector<std::string> adapter_namespaces = 
      hardware_interface_adapter::internal::getStrings(controller_nh, "adapter_namespaces");
    if(adapter_namespaces.size() != 2) {
      ROS_ERROR_STREAM_NAMED(ctrl_name, 
          "The parameter '" << controller_nh.getNamespace() + "/adapter_namespaces" <<
          "' must be set with a list of 2 joint adapter names used by this controller.");
      return false;
    }

    joint_adapters = 
      hardware_interface_adapter::internal::getStrings(controller_nh, "joint_adapters");
    if(joint_adapters.size() != joint_names.size()) {
      ROS_ERROR_STREAM_NAMED(ctrl_name, 
          "The parameter '" << controller_nh.getNamespace() + "/joint_adapters" <<
          "' must be set with a list of joint adapters corresponding to each joint.");
      return false;
    }

    // Initialize members
    joint_states.resize(n_joints);
    for (unsigned int i = 0; i < n_joints; ++i) {
      std::string adapter = joint_adapters[i];
      int adapter_ind = std::find(adapter_namespaces.begin(), adapter_namespaces.end(), adapter) - 
                        adapter_namespaces.begin();
      if(adapter_ind == adapter_namespaces.size()) {
        ROS_ERROR_STREAM_NAMED(ctrl_name, 
            "Could not find adapter named '" << adapter <<
            "'. Available adapters: " << 
            "'" << adapter_namespaces[0] << "', " <<
            "'" << adapter_namespaces[1] << "'");
        return false;
      }
      adapter_inds.push_back(adapter_ind);
      try {
        switch(adapter_ind) {
          case 0:
            adapter1.joint_handles.push_back(hw1->getHandle(joint_names[i]));
            joint_states[i] = static_cast<hardware_interface::JointStateHandle>
                                                        (adapter1.joint_handles.back());
            break;
          case 1:
            adapter2.joint_handles.push_back(hw2->getHandle(joint_names[i]));
            joint_states[i] = static_cast<hardware_interface::JointStateHandle>
                                                        (adapter2.joint_handles.back());
            break;
        }
      }
      catch (...) {
        ROS_ERROR_STREAM_NAMED(ctrl_name, "Could not find joint '" << joint_names[i] << 
                               "' in '" << adapter_namespaces[adapter_ind] << "'.");
        return false;
      }
    }

    ros::NodeHandle controller_nh1(controller_nh, adapter_namespaces[0]);
    ros::NodeHandle controller_nh2(controller_nh, adapter_namespaces[1]);
    if(!adapter1.init(controller_nh1))
      return false;
    if(!adapter2.init(controller_nh2))
      return false;

    desired_state1 = State(adapter1.joint_handles.size());
    desired_state2 = State(adapter2.joint_handles.size());
    state_error1 = State(adapter1.joint_handles.size());
    state_error2 = State(adapter2.joint_handles.size());

    ROS_DEBUG_STREAM_NAMED(ctrl_name, 
        "Initialized adapter for controller '" << ctrl_name << "' with:" <<
        "\n- Number of joints: " << n_joints <<
        "\n- Hardware adapter names: " <<
        "'" << adapter_namespaces[0] << "' (" << 
        hardware_interface::internal::demangledTypeName<HwIfaceAdapter1>() << "), '" <<
        "'" << adapter_namespaces[1] << "' (" << 
        hardware_interface::internal::demangledTypeName<HwIfaceAdapter2>() << ")" <<
        "\n- State type: '" << hardware_interface::internal::demangledTypeName<State>() << "'");

    return true;
  }

  virtual void starting(const ros::Time& time) 
  {
    adapter1.starting(time);
    adapter2.starting(time);
  }

  virtual void stopping(const ros::Time& time) 
  {
    adapter1.stopping(time);
    adapter2.stopping(time);
  }

  virtual void updateCommand(const ros::Time&     time,
                             const ros::Duration& period,
                             const State&         desired_state,
                             const State&         state_error) 
  {
    int ind1 = 0, ind2 = 0;
    for(int i = 0; i < adapter_inds.size(); ++i) {
      if(adapter_inds[i] == 0) {
        desired_state1.position[ind1] = desired_state.position[i];
        desired_state1.velocity[ind1] = desired_state.velocity[i];
        desired_state1.acceleration[ind1] = desired_state.acceleration[i];
        state_error1.position[ind1] = state_error.position[i];
        state_error1.velocity[ind1] = state_error.velocity[i];
        state_error1.acceleration[ind1] = state_error.acceleration[i];
        ind1++;
      } 
      else if(adapter_inds[i] == 1) {
        desired_state2.position[ind2] = desired_state.position[i];
        desired_state2.velocity[ind2] = desired_state.velocity[i];
        desired_state2.acceleration[ind2] = desired_state.acceleration[i];
        state_error2.position[ind2] = state_error.position[i];
        state_error2.velocity[ind2] = state_error.velocity[i];
        state_error2.acceleration[ind2] = state_error.acceleration[i];
        ind2++;
      }
      else
        assert(false);
    }
    adapter1.updateCommand(time, period, desired_state1, state_error1);
    adapter2.updateCommand(time, period, desired_state2, state_error2);
  }
};

/**
 * \brief Adapter for a position-controlled hardware interface. Forwards desired positions as commands.
 *
 * The following is an example configuration of a controller that uses this adapter.
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
 *     head_1_joint: {trajectory: 0.05, goal: 0.02}
 *     head_2_joint: {trajectory: 0.05, goal: 0.02}
 *   stop_trajectory_duration: 0.5
 *   state_publish_rate:  25
 * \endcode
 */
template <class State>
class PositionHardwareInterfaceAdapter 
  : public HardwareInterfaceAdapter<hardware_interface::PositionJointInterface, State>
{
public:

  bool init(ros::NodeHandle& controller_nh)
  {
    return true;
  }

  void updateCommand(const ros::Time&     /*time*/,
                     const ros::Duration& /*period*/,
                     const State&         desired_state,
                     const State&         /*state_error*/)
  {
    // Forward desired position to command
    for (unsigned int i = 0; i < this->joint_handles.size(); ++i) 
      this->joint_handles[i].setCommand(desired_state.position[i]);
  }

};

/**
 * \brief Adapter for an effort-controlled hardware interface. Maps position and velocity errors to effort commands
 * through a position PID loop.
 *
 * The following is an example configuration of a controller that uses this adapter. Notice the \p gains entry:
 * \code
 * head_controller:
 *   type: "effort_controllers/JointTrajectoryController"
 *   joints:
 *     - head_1_joint
 *     - head_2_joint
 *   gains:
 *     head_1_joint: {p: 200, d: 1, i: 5, i_clamp: 1}
 *     head_2_joint: {p: 200, d: 1, i: 5, i_clamp: 1}
 *   constraints:
 *     goal_time: 0.6
 *     stopped_velocity_tolerance: 0.02
 *     head_1_joint: {trajectory: 0.05, goal: 0.02}
 *     head_2_joint: {trajectory: 0.05, goal: 0.02}
 *   stop_trajectory_duration: 0.5
 *   state_publish_rate:  25
 * \endcode
 */
template <class State>
class EffortHardwareInterfaceAdapter 
  : public HardwareInterfaceAdapter<hardware_interface::EffortJointInterface, State>
{
public:

  bool init(ros::NodeHandle& controller_nh)
  {
    // Initialize PIDs
    pids_.resize(this->joint_handles.size());
    for (unsigned int i = 0; i < pids_.size(); ++i)
    {
      // Node handle to PID gains
      ros::NodeHandle joint_nh(controller_nh, std::string("gains/") + this->joint_handles[i].getName());

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
    // Reset PIDs, zero effort commands
    for (unsigned int i = 0; i < pids_.size(); ++i)
    {
      pids_[i]->reset();
      this->joint_handles[i].setCommand(0.0);
    }
  }

  void updateCommand(const ros::Time&     /*time*/,
                     const ros::Duration& period,
                     const State&         /*desired_state*/,
                     const State&         state_error)
  {
    const unsigned int n_joints = this->joint_handles.size();

    // Preconditions
    assert(n_joints == state_error.position.size());
    assert(n_joints == state_error.velocity.size());

    // Update PIDs
    for (unsigned int i = 0; i < n_joints; ++i)
    {
      const double command = pids_[i]->computeCommand(state_error.position[i], 
                                                      state_error.velocity[i], period);
      this->joint_handles[i].setCommand(command);
    }
  }

private:
  typedef boost::shared_ptr<control_toolbox::Pid> PidPtr;
  std::vector<PidPtr> pids_;
};

/**
 * \brief Adapter for a position-controlled hardware interface. Forwards desired positions as commands.
 */
template <class State>
class PosVelAccForwardHardwareInterfaceAdapter 
  : public HardwareInterfaceAdapter<hardware_interface::PosVelAccJointInterface, State>
{
public:

  bool init(ros::NodeHandle& controller_nh)
  {
    return true;
  }

  void updateCommand(const ros::Time&     /*time*/,
                     const ros::Duration& /*period*/,
                     const State&         desired_state,
                     const State&         /*state_error*/)
  {
    // Forward desired position to command
    for (unsigned int i = 0; i < this->joint_handles.size(); ++i) {
      this->joint_handles[i].setPositionCommand(desired_state.position[i]);
      this->joint_handles[i].setVelocityCommand(desired_state.velocity[i]);
      this->joint_handles[i].setAccelerationCommand(desired_state.acceleration[i]);
    }
  }
};

/**
 * \brief Adapter for a position-controlled hardware interface. Forwards desired positions as commands.
 */
template <class State>
class VelocityForwardHardwareInterfaceAdapter 
  : public HardwareInterfaceAdapter<hardware_interface::VelocityJointInterface, State>
{
public:

  bool init(ros::NodeHandle& controller_nh)
  {
    // zero velocity commands
    for (unsigned int i = 0; i < this->joint_handles.size(); ++i)
      this->joint_handles[i].setCommand(0.0);

    return true;
  }

  void starting(const ros::Time& time) 
  {
    // zero velocity commands
    for (unsigned int i = 0; i < this->joint_handles.size(); ++i)
      this->joint_handles[i].setCommand(0.0);
  }

  void stopping(const ros::Time& time) 
  {
    // zero velocity commands
    for (unsigned int i = 0; i < this->joint_handles.size(); ++i)
      this->joint_handles[i].setCommand(0.0);
  }

  void updateCommand(const ros::Time&     /*time*/,
                     const ros::Duration& /*period*/,
                     const State&         desired_state,
                     const State&         /*state_error*/)
  {
    // Forward desired position to command
    for (unsigned int i = 0; i < this->joint_handles.size(); ++i)
      this->joint_handles[i].setCommand(desired_state.velocity[i]);
  }
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

  bool init(ros::NodeHandle& controller_nh)
  {
    const int n_joints = this->joint_handles.size();
    double cmd_publish_rate;
    controller_nh.param<double>("cmd_publish_rate", cmd_publish_rate, 100.0);
    cmd_pub_period_ = ros::Duration(1.0 / cmd_publish_rate);

    // Initialize PIDs
    pids_.resize(n_joints);
    for (unsigned int i = 0; i < n_joints; ++i)
    {
      // Node handle to PID gains
      ros::NodeHandle joint_nh(controller_nh, std::string("gains/") + this->joint_handles[i].getName());

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
      for (unsigned int i = 0; i < n_joints; ++i) 
        command_publisher_->msg_.data.push_back(this->joint_handles[i].getCommand());
      command_publisher_->msg_.layout.dim.resize(1);
      command_publisher_->msg_.layout.dim[0].size = n_joints;
      command_publisher_->unlock();
    }

    return true;
  }

  void starting(const ros::Time& time)
  {
    // Reset PIDs, zero velocity commands
    for (unsigned int i = 0; i < pids_.size(); ++i)
    {
      pids_[i]->reset();
      this->joint_handles[i].setCommand(0.0);
    }
  }

  void stopping(const ros::Time& time) 
  {
    // zero velocity commands
    for (unsigned int i = 0; i < this->joint_handles.size(); ++i)
      this->joint_handles[i].setCommand(0.0);
  }

  void updateCommand(const ros::Time&     time,
                     const ros::Duration& period,
                     const State&         /*desired_state*/,
                     const State&         state_error)
  {
    const unsigned int n_joints = this->joint_handles.size();

    // Preconditions
    assert(n_joints == state_error.position.size());
    assert(n_joints == state_error.velocity.size());

    // Update commands from PIDs
    for (unsigned int i = 0; i < n_joints; ++i)
    {
      const double vel_command = pids_[i]->computeCommand(state_error.position[i], 
                                                          state_error.velocity[i], 
                                                          period);
      this->joint_handles[i].setCommand(vel_command);
    }

    if (!cmd_pub_period_.isZero() && last_cmd_pub_time_ + cmd_pub_period_ < time)
    {
      if (command_publisher_ && command_publisher_->trylock())
      {
        last_cmd_pub_time_ += cmd_pub_period_;
        for (unsigned int i = 0; i < n_joints; ++i)
          command_publisher_->msg_.data[i] = this->joint_handles[i].getCommand();
        command_publisher_->unlockAndPublish();
      }
    }
  }

private:
  typedef boost::shared_ptr<control_toolbox::Pid> PidPtr;
  std::vector<PidPtr> pids_;

  typedef realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray> CommandPublisher;
  typedef boost::scoped_ptr<CommandPublisher> CommandPublisherPtr;
  CommandPublisherPtr command_publisher_;
  ros::Time last_cmd_pub_time_;
  ros::Duration cmd_pub_period_;
};

#endif // header guard
