
#ifndef UR_FORWARD_CMD_CONTROLLERS_H
#define UR_FORWARD_CMD_CONTROLLERS_H

#include <ros/node_handle.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

// URDF
#include <urdf/model.h>

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Empty.h>

#include <joint_trajectory_controller/hardware_interface_adapter.h>

using hardware_interface::JointHandle;
using trajectory_interface::PosVelAccState;
using namespace joint_trajectory_controller::internal;

namespace jogging_command_controller
{

template <class HardwareInterface>
class JoggingCommandController: 
  public controller_interface::Controller<HardwareInterface>
{
public:
  JoggingCommandController() {}
  ~JoggingCommandController() {sub_command_.shutdown();}

  bool init(HardwareInterface* hw, ros::NodeHandle &root_nh, ros::NodeHandle &ctrl_nh);
  void starting(const ros::Time& time);
  void update(const ros::Time& time, const ros::Duration& period);

private:
  PosVelAccState current_state_;
  PosVelAccState desired_state_;
  PosVelAccState state_error_;  

  std::vector<hardware_interface::JointHandle> joints_;
  std::vector<double> lower_limits_;
  std::vector<double> upper_limits_;

};


template <class HardwareInterface>
bool JoggingCommandController<HardwareInterface>::
init(HardwareInterface* hw, ros::NodeHandle &root_nh, ros::NodeHandle &ctrl_nh)
{

  // List of controlled joints
  joint_names_ = getStrings(ctrl_nh, "joints");
  if (joint_names_.empty()) {return false;}
  const unsigned int n_joints = joint_names_.size();

  // URDF joints
  boost::shared_ptr<urdf::Model> urdf = getUrdf(root_nh, "robot_description");
  if (!urdf) {return false;}

  std::vector<UrdfJointConstPtr> urdf_joints = getUrdfJoints(*urdf, joint_names_);
  if (urdf_joints.empty()) {return false;}
  assert(n_joints == urdf_joints.size());

  // Initialize members
  joints_.resize(n_joints);
  for (unsigned int i = 0; i < n_joints; ++i) {
    // Joint handle
    try {joints_[i] = hw->getHandle(joint_names_[i]);}
    catch (...) {
      ROS_ERROR_STREAM_NAMED(name_, "Could not find joint '" << joint_names_[i] << "' in '" <<
                                    this->getHardwareInterfaceType() << "'.");
      return false;
    }
    if(urdf_joints[i].limits) {
      lower_limits_.push_back(urdf_joints[i].limits.lower);
      upper_limits_.push_back(urdf_joints[i].limits.upper);
    } else {
      lower_limits_.push_back(-std::numeric_limits<double>::max());
      upper_limits_.push_back(std::numeric_limits<double>::max());
    }
  }

  // Hardware interface adapter
  hw_iface_adapter_.init(joints_, controller_nh_);
}


template <class HardwareInterface>
bool JoggingCommandController<HardwareInterface>::
void starting(const ros::Time& time)
{
}

template <class HardwareInterface>
bool JoggingCommandController<HardwareInterface>::
void update(const ros::Time& time, const ros::Duration& period)
{

  // Hardware interface adapter: Generate and send commands
  hw_iface_adapter_.updateCommand(time, period,
                                  desired_state_, state_error_);
}








{

  std::vector<double> vel_;
  ros::Time last_cmd_time_;
  int reset_vel_;
  double reset_timeout_;
  bool has_updated_;

  ros::Subscriber sub_command_;
  void commandCB(const std_msgs::Float64MultiArrayConstPtr& msg) 
  {
    if(msg->data.size() != vel_.size()) {
      ROS_WARN("VelocityForwardController: velocities size %d not of same length as joint_names %d",
               (int) msg->data.size(), (int) vel_.size());
      return;
    }
    for(int i=0;i<joints_.size();i++) {
      vel_[i] = msg->data[i];
    }
    has_updated_ = true;
  }
}




  if(!n.hasParam("joint_names")) {
    ROS_ERROR("VelocityForwardController: parameter joint_names must be defined");
    return false;
  }
  XmlRpc::XmlRpcValue v;
  n.param("joint_names", v, v);
  for(int i=0;i<v.size();i++)
    joints_.push_back(hw->getHandle(v[i]));
  vel_.resize(joints_.size());
  n.param("reset_vel", reset_vel_, 0);
  n.param("reset_timeout", reset_timeout_, 1.0);

  sub_command_ = n.subscribe<std_msgs::Float64MultiArray>("command", 1, 
                                      &VelocityForwardController::commandCB, this);
  return true;
}

}

#endif
