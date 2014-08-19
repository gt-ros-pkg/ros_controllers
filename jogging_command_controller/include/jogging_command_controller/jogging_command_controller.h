
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

template <class ParamType>
std::vector<ParamType> getParamList(const XmlRpcValue::Type xml_rpc_val_type, 
                                    const ros::NodeHandle& nh, const std::string& param_name)
{
  using namespace XmlRpc;
  XmlRpcValue xml_array;
  if (!nh.getParam(param_name, xml_array))
  {
    ROS_ERROR_STREAM("Could not find '" << param_name << "' parameter (namespace: " << nh.getNamespace() << ").");
    return std::vector<ParamType>();
  }
  if (xml_array.getType() != XmlRpcValue::TypeArray)
  {
    ROS_ERROR_STREAM("The '" << param_name << "' parameter is not an array (namespace: " <<
                     nh.getNamespace() << ").");
    return std::vector<ParamType>();
  }

  std::vector<ParamType> out;
  for (int i = 0; i < xml_array.size(); ++i)
  {
    if (xml_array[i].getType() != xml_rpc_val_type)
    {
      ROS_ERROR_STREAM("The '" << param_name << "' parameter contains a non-string element (namespace: " <<
                       nh.getNamespace() << ").");
      return std::vector<ParamType>();
    }
    out.push_back(static_cast<ParamType>(xml_array[i]));
  }
  return out;
}

template <class HardwareInterface>
class JoggingCommandController : 
  public controller_interface::Controller<HardwareInterface>
{
public:
  JoggingCommandController() {}
  ~JoggingCommandController() {sub_command_.shutdown();}

  bool init(HardwareInterface* hw, ros::NodeHandle &root_nh, ros::NodeHandle &ctrl_nh);
  void starting(const ros::Time& time);
  void update(const ros::Time& time, const ros::Duration& period);

private:
  
  ros::NodeHandle controller_nh_;
  std::string name_;

  PosVelAccState current_state_;
  PosVelAccState desired_state_;
  PosVelAccState state_error_;  

  std::vector<hardware_interface::JointHandle> joints_;
  std::vector<double> lower_pos_limits_;
  std::vector<double> upper_pos_limits_;
  std::vector<double> velocity_limits_;

  std::vector<ros::Subscriber> command_subs_;

  std::vector<bool> heartbeat_updated_;
  std::vector<ros::Time> heartbeat_last_time_;
  double heartbeat_timeout_;

  std::vector<double> jogging_commands_;
  std::vector<bool> stop_jogging_;
  std::vector<double> jogging_acceleration_;
  std::vector<double> jogging_deceleration_;
};


template <class HardwareInterface>
bool JoggingCommandController<HardwareInterface>::
init(HardwareInterface* hw, ros::NodeHandle &root_nh, ros::NodeHandle &ctrl_nh)
{
  double MAX_DOUBLE = std::numeric_limits<double>::max();
  // Cache controller node handle
  controller_nh_ = controller_nh;

  // Controller name
  name_ = getLeafNamespace(controller_nh_);

  heartbeat_timeout_ = 0.3;
  controller_nh_.getParam("heartbeat_timeout", heartbeat_timeout_);
  ROS_DEBUG_STREAM_NAMED(name_, "Commands timeout after " << heartbeat_timeout_ 
                                << "sec without receiving identical command.");

  // List of controlled joints
  joint_names_ = getParamList<std::string>(XmlRpcValue::TypeString, ctrl_nh, "joints");
  if (joint_names_.empty()) {return false;}
  const unsigned int n_joints = joint_names_.size();

  // URDF joints
  boost::shared_ptr<urdf::Model> urdf = getUrdf(root_nh, "robot_description");
  if (!urdf) {return false;}

  std::vector<UrdfJointConstPtr> urdf_joints = getUrdfJoints(*urdf, joint_names_);
  if (urdf_joints.empty()) {return false;}
  assert(n_joints == urdf_joints.size());

  double default_acceleration = -1.0, default_deceleration = -1.0;
  controller_nh_.getParam("acceleration", default_acceleration);
  controller_nh_.getParam("deceleration", default_deceleration);

  // Initialize members
  joints_.resize(n_joints);
  lower_pos_limits_.resize(n_joints);
  upper_pos_limits_.resize(n_joints);
  velocity_limits_.resize(n_joints);
  heartbeat_updated_.resize(n_joints, false);
  heartbeat_last_time_.resize(n_joints, ros::Time());
  jogging_commands_.resize(n_joints, 0.0);
  stop_jogging_.resize(n_joints, false);
  jogging_acceleration_.resize(n_joints);
  jogging_deceleration_.resize(n_joints);
  for (unsigned int i = 0; i < n_joints; ++i) {
    // Joint handle
    try {joints_[i] = hw->getHandle(joint_names_[i]);}
    catch (...) {
      ROS_ERROR_STREAM_NAMED(name_, "Could not find joint '" << joint_names_[i] << "' in '" <<
                                    this->getHardwareInterfaceType() << "'.");
      return false;
    }
    ros::NodeHandle joint_nh(ctrl_nh, joint_names_[i]);

    // retrieve jogging acceleration for joint, but if no default and no
    // individual parameter exists, we cannot start the controller
    jogging_acceleration_[i] = default_acceleration;
    joint_nh.getParam("acceleration", jogging_acceleration_[i]);
    if(jogging_acceleration_[i] <= 0.0) {
      ROS_DEBUG_STREAM_NAMED(name_, "Must provide positive acceleration for joint " 
                                        << joint_names_[i] << ".");
      return false;
    }

    // the deceleration looks first for the specific, then at the default, 
    // then at the acceleration for that joint
    // this parameter is thus totally optional
    jogging_deceleration_[i] = (default_deceleration > 0.0) ?
                                default_deceleration : jogging_acceleration_[i];
    joint_nh.getParam("deceleration", jogging_deceleration_[i]);

    // grab limits from parameter first, then URDF
    lower_pos_limits_[i] = (urdf_joints[i].limits) ? urdf_joints[i].limits.lower : -MAX_DOUBLE;
    joint_nh.getParam("lower_pos_limit", lower_pos_limit_[i]);
    upper_pos_limits_[i] = (urdf_joints[i].limits) ? urdf_joints[i].limits.upper : MAX_DOUBLE;
    joint_nh.getParam("upper_pos_limit", upper_pos_limit_[i]);
    velocity_limits_[i] = (urdf_joints[i].limits) ? urdf_joints[i].limits.velocity : MAX_DOUBLE;
    joint_nh.getParam("velocity_limit",  velocity_limit_[i]);

    command_subs_.push_back(ctrl_nh.subscribe<std_msgs::Float64>(joint_names_[i] + "/command", 1, 
            boost::bind(boost::bind(&JoggingCommandController::commandCB, this, _1), i, _1)));
  }

  current_state_    = PosVelAccState(n_joints);
  desired_state_    = PosVelAccState(n_joints);
  state_error_      = PosVelAccState(n_joints);

  // Hardware interface adapter
  hw_iface_adapter_.init(joints_, controller_nh_);
}

template <class HardwareInterface>
bool JoggingCommandController<HardwareInterface>::
void commandCB(int joint_index, const std_msgs::Float64::ConstPtr& cmd)
{
  if(cmd.data == 0.0) {
    // zero command means stop jogging
    stop_jogging_[joint_index] = true;
    return;
  }

  if(jogging_commands_[joint_index] == 0.0) {
    // jogging not started, start jogging this joint

    // set jogging target velocity to command clipped to velocity limit
    jogging_commands_[joint_index] = 
      std::min( std::max(cmd.data, -velocity_limits_[joint_index]), 
                                    velocity_limits_[joint_index]);
    stop_jogging_[joint_index] = false;

    desired_state_.position[joint_index] = joints_[joint_index].getPosition();
    desired_state_.velocity[joint_index] = 0.0;
    if(cmd.data > 0.0)
      desired_state_.acceleration[joint_index] = jogging_acceleration_[joint_index];
    else
      desired_state_.acceleration[joint_index] = -jogging_acceleration_[joint_index];

    heartbeat_updated_[joint_index] = true;

  } else {

    if(jogging_commands_[joint_index] == cmd.data) {
      // we have a consistent heartbeat command, continue jogging
      if(!stop_jogging_[joint_index]) {
        heartbeat_updated_[joint_index] = true;
      }
    } else {
      // we have an inconsistent command, stop jogging
      stop_jogging_[joint_index] = true;
    }
  }
}


template <class HardwareInterface>
bool JoggingCommandController<HardwareInterface>::
void starting(const ros::Time& time)
{
  // Hardware interface adapter
  hw_iface_adapter_.starting(time_data.uptime);
}

template <class HardwareInterface>
bool JoggingCommandController<HardwareInterface>::
void update(const ros::Time& time, const ros::Duration& period)
{
  for (unsigned int i = 0; i < joints_.size(); ++i) {
    if(jogging_commands_[i] != 0.0) {
      // jogging activated

      // check for heartbeat timeout
      if(heartbeat_updated_[i])
        heartbeat_last_time_[i] = time;
      if((time - heartbeat_last_time_[i]).toSec() > heartbeat_timeout_)
        stop_jogging_[i] = true;

      // check for being close to joint limits
      double time_to_stop = desired_state_.velocity[i] / jogging_deceleration_[i];
      // dist_to_stop is always positive
      double dist_to_stop = 0.5 * jogging_deceleration_[i] * time_to_stop * time_to_stop;
      // if we're getting close to the place where full deceleration will stop the
      // robot just short of the limits, then we go ahead and stop
      if(desired_state_.velocity[i] > 0.0) {
        if(joints_[i].getPosition() + 1.2*dist_to_stop > upper_pos_limits_[i])
          stop_jogging_[i] = true;
      } else {
        if(joints_[i].getPosition() - 1.2*dist_to_stop < lower_pos_limits_[i])
          stop_jogging_[i] = true;
      }

      if(stop_jogging_[i]) {
        // begin decelerating
        if(desired_state_.velocity[i] > 0.0)
          desired_state_.acceleration[i] = -jogging_deceleration_[i];
        else
          desired_state_.acceleration[i] = jogging_deceleration_[i];
      }

      // update velocity
      desired_state_.velocity[i] += desired_state_.acceleration[i]*period.toSec();

      // check whether acceleration needs update
      if(!stop_jogging_[i]) {
        // jogging activated and not stopping

        if(std::fabs(desired_state_.velocity[i]) >= std::fabs(jogging_commands_[i])) {
          // reached jogging speed, cruise at this speed
          desired_state_.velocity[i] = jogging_commands_[i];
          desired_state_.acceleration[i] = 0.0;
        }
      } else {
        // jogging activated but decelerating 
        
        if(desired_state_.velocity[i]*jogging_commands_[i] < 0.0) {
          // velocity has flipped signs, completely stop now
          desired_state_.velocity[i] = 0.0;
          desired_state_.acceleration[i] = 0.0;
          jogging_commands_[i] = 0.0; // jogging command stopped
        }
      }

      // update position
      desired_state_.position[i] += desired_state_.velocity[i]*period.toSec();
    } else {
      // not moving
      desired_state_.position[i] = joints_[i].getPosition();
      desired_state_.velocity[i] = 0.0;
      desired_state_.acceleration[i] = 0.0;
    }

    current_state_.position[i] = joints_[i].getPosition();
    current_state_.velocity[i] = joints_[i].getVelocity();
    // There's no acceleration data available in a joint handle

    state_error_.position[i] = desired_state_.position[i] - current_state_.position[i];
    state_error_.velocity[i] = desired_state_.velocity[i] - current_state_.velocity[i];
    state_error_.acceleration[i] = 0.0;
  }

  // Hardware interface adapter: Generate and send commands
  hw_iface_adapter_.updateCommand(time, period,
                                  desired_state_, state_error_);
}

}

#endif
