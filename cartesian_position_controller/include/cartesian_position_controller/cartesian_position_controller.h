
#ifndef CARTESIAN_POSITION_CONTROLLER_H
#define CARTESIAN_POSITION_CONTROLLER_H

#include <boost/thread.hpp>

#include <ros/node_handle.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <controller_interface/multi_controller.h>

// URDF
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolvervel_pinv_givens.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/chainiksolvervel_pinv_nso.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>

#include <tf/transform_datatypes.h>

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <joint_trajectory_controller/joint_trajectory_controller.h>
#include <joint_trajectory_controller/joint_trajectory_controller_impl.h>

using namespace joint_trajectory_controller::internal;
using namespace XmlRpc;

namespace cartesian_position_controller
{

template <class ParamType>
std::vector<ParamType> getParamList(const XmlRpcValue::Type xml_rpc_val_type, 
                                    const ros::NodeHandle& nh, const std::string& param_name)
{
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

template <class State, class HwIfaceAdapter, class Controller>
class CartesianPositionControllerBase : public Controller
{
public:
  CartesianPositionControllerBase() {}
  ~CartesianPositionControllerBase() 
  {}

  bool initInternal(ros::NodeHandle &root_nh, ros::NodeHandle &ctrl_nh);
  void starting(const ros::Time& time);
  void update(const ros::Time& time, const ros::Duration& period);

protected:
  // void commandCB(int joint_index, const std_msgs::Float64::ConstPtr& cmd);
  void setOptPos(const std_msgs::Float64MultiArray::ConstPtr& msg)
  { 
    boost::lock_guard<boost::mutex> lock(mutex_lock_);
    boost::shared_ptr<KDL::JntArray> q = multiArrayToJntArray(msg);
    if(q)
      boost::static_pointer_cast<KDL::ChainIkSolverVel_pinv_nso>(ik_vel_solver_)->setOptPos(*q); 
  }
  void setWeights(const std_msgs::Float64MultiArray::ConstPtr& msg)
  { 
    boost::lock_guard<boost::mutex> lock(mutex_lock_);
    boost::shared_ptr<KDL::JntArray> q = multiArrayToJntArray(msg);
    if(q)
      boost::static_pointer_cast<KDL::ChainIkSolverVel_pinv_nso>(ik_vel_solver_)->setWeights(*q);
  }
  void setWeightJS(const std_msgs::Float64MultiArray::ConstPtr& msg)
  { 
    boost::lock_guard<boost::mutex> lock(mutex_lock_);
    boost::shared_ptr<Eigen::MatrixXd> Mq = multiArrayToDiagMatrix(msg);
    if(Mq)
      boost::static_pointer_cast<KDL::ChainIkSolverVel_wdls>(ik_vel_solver_)->setWeightJS(*Mq);
  }
  void setWeightTS(const std_msgs::Float64MultiArray::ConstPtr& msg)
  { 
    boost::lock_guard<boost::mutex> lock(mutex_lock_);
    boost::shared_ptr<Eigen::MatrixXd> Mx = multiArrayToDiagMatrix(msg);
    if(Mx)
      boost::static_pointer_cast<KDL::ChainIkSolverVel_wdls>(ik_vel_solver_)->setWeightTS(*Mx);
  }

  boost::shared_ptr<KDL::JntArray> multiArrayToJntArray(
      const std_msgs::Float64MultiArray::ConstPtr& msg)
  {
    int num_jnts = kdl_chain_.getNrOfJoints();
    if(num_jnts != msg->data.size()) {
      ROS_ERROR_NAMED(name_, "Message not correct length (%d)", num_jnts);
      return boost::shared_ptr<KDL::JntArray>();
    }
    boost::shared_ptr<KDL::JntArray> q(new KDL::JntArray(num_jnts));
    for(int i = 0; i < num_jnts; ++i)
      q->data(i) = msg->data[i];
    return q;
  }

  boost::shared_ptr<Eigen::MatrixXd> multiArrayToDiagMatrix(
      const std_msgs::Float64MultiArray::ConstPtr& msg)
  {
    int num_jnts = kdl_chain_.getNrOfJoints();
    if(num_jnts != msg->data.size()) {
      ROS_ERROR_NAMED(name_, "Message not correct length (%d)", num_jnts);
      return boost::shared_ptr<Eigen::MatrixXd>();
    }
    boost::shared_ptr<Eigen::MatrixXd> mat(new Eigen::MatrixXd(num_jnts, num_jnts));
    for(int i = 0; i < num_jnts; ++i)
      (*mat)(i, i) = msg->data[i];
    return mat;
  }

  void commandPose(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    x_des_ = tf::Vector3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    quat_des_ = tf::Quaternion(msg->pose.orientation.x, msg->pose.orientation.y, 
                               msg->pose.orientation.z, msg->pose.orientation.w);
  }

  void commandTwist(const geometry_msgs::TwistStamped::ConstPtr& msg)
  {
    lin_fwd_vel_ = tf::Vector3(msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z);
    ang_fwd_vel_ = tf::Vector3(msg->twist.angular.x, msg->twist.angular.y, msg->twist.angular.z);
  }
  
  ros::NodeHandle ctrl_nh_;
  std::string name_;
  boost::mutex mutex_lock_;

  HwIfaceAdapter hw_iface_adapter_;   ///< Adapts desired state to HW interface.

  State current_state_;
  State desired_state_;
  State state_error_;  

  std::vector<hardware_interface::JointStateHandle> joint_states_;
  std::vector<std::string> joint_names_;

  // KDL kinematics solvers
  KDL::Chain kdl_chain_;
  boost::shared_ptr<KDL::ChainIkSolverVel> ik_vel_solver_;
  boost::shared_ptr<KDL::ChainFkSolverPos> fk_pos_solver_;
  boost::shared_ptr<KDL::ChainFkSolverVel> fk_vel_solver_;

  // controller internal states
  KDL::JntArrayVel jnt_states_cur_;
  KDL::Frame x_cur_;
  KDL::FrameVel xd_cur_frame_vel_;
  KDL::Twist xd_cmd_;
  KDL::JntArray qd_cmd_;
  KDL::JntArray qd_cmd_last_;
  bool first_update_;

  // parameters for controller dynamics
  tf::Vector3 linear_cmd_gains_, linear_cmd_maxs_;
  tf::Vector3 angle_cmd_gains_, angle_cmd_maxs_;
  std::vector<double> qd_abs_maxs_;
  std::vector<double> qdd_abs_maxs_;

  // controller commands
  tf::Vector3 x_des_;
  tf::Quaternion quat_des_;
  // forward velocity commands
  tf::Vector3 lin_fwd_vel_;
  tf::Vector3 ang_fwd_vel_;
  
  // controller command subscribers
  ros::Subscriber command_pose_sub_;
  ros::Subscriber command_twist_sub_;

  // IK solver parameter subscribers
  ros::Subscriber opt_pos_sub_;
  ros::Subscriber weights_sub_;
  ros::Subscriber weight_js_sub_;
  ros::Subscriber weight_ts_sub_;
};

template <class State, class HwIfaceAdapter>
class CartesianPositionController
  : public CartesianPositionControllerBase<State, HwIfaceAdapter, 
                    controller_interface::Controller<typename HwIfaceAdapter::HwIface> >
{
public:
  typedef typename HwIfaceAdapter::HwIface HardwareInterface;

  bool init(HardwareInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
};

template <class State, class HwIfaceAdapter>
bool CartesianPositionController<State, HwIfaceAdapter>::
init(HardwareInterface* hw, ros::NodeHandle &root_nh, ros::NodeHandle &ctrl_nh)
{
  if(!this->initInternal(root_nh, ctrl_nh))
    return false;

  if(!this->hw_iface_adapter_.initJoints(this->name_, this->joint_names_, hw, 
                                         ctrl_nh, this->joint_states_))
      return false;

  return true;
}

template <class State, class HwIfaceAdapter>
class CartesianPositionController2
  : public CartesianPositionControllerBase<State, HwIfaceAdapter, 
                    controller_interface::Controller2<typename HwIfaceAdapter::HwIface1,
                                                      typename HwIfaceAdapter::HwIface2> >
{
public:
  typedef typename HwIfaceAdapter::HwIface1 HardwareInterface1;
  typedef typename HwIfaceAdapter::HwIface2 HardwareInterface2;

  bool init(HardwareInterface1* hw1, HardwareInterface2* hw2, 
            ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
};

template <class State, class HwIfaceAdapter>
bool CartesianPositionController2<State, HwIfaceAdapter>::
init(HardwareInterface1* hw1, HardwareInterface2* hw2, 
     ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  if(!this->initInternal(root_nh, controller_nh))
    return false;

  if(!this->hw_iface_adapter_.initJoints(this->name_, this->joint_names_, hw1, hw2,
                                         controller_nh, this->joint_states_))
      return false;

  return true;
}

}

#include <cartesian_position_controller/cartesian_position_controller_impl.h>

#endif
