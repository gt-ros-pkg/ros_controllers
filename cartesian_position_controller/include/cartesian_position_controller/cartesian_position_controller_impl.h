#ifndef CARTESIAN_POSITION_CONTROLLER_IMPL_H
#define CARTESIAN_POSITION_CONTROLLER_IMPL_H

namespace cartesian_position_controller
{

template <class State, class HwIfaceAdapter, class Controller>
bool CartesianPositionControllerBase<State, HwIfaceAdapter, Controller>::
initInternal(ros::NodeHandle &root_nh, ros::NodeHandle &ctrl_nh)
{
  // Cache controller node handle
  ctrl_nh_ = ctrl_nh;

  // Controller name
  name_ = getLeafNamespace(ctrl_nh_);

  // List of controlled joints
  joint_names_ = getParamList<std::string>(XmlRpcValue::TypeString, ctrl_nh, "joints");
  if (joint_names_.empty()) {
    ROS_ERROR_NAMED(name_, "Could not find controller parameter 'joints'");
    return false;
  }
  const unsigned int num_jnts = joint_names_.size();

  // URDF joints
  boost::shared_ptr<urdf::Model> urdf = getUrdf(root_nh, "robot_description");
  if (!urdf) {
    ROS_ERROR_NAMED(name_, "Could not create URDF");
    return false;
  }

  std::vector<UrdfJointConstPtr> urdf_joints = getUrdfJoints(*urdf, joint_names_);
  if (urdf_joints.empty()) {
    ROS_ERROR_NAMED(name_, "Could not find parameter joints in URDF");
    return false;
  }
  assert(num_jnts == urdf_joints.size());

  KDL::Tree kdl_tree;
  if(!kdl_parser::treeFromUrdfModel(*urdf, kdl_tree)) {
    ROS_ERROR_NAMED(name_, "Could not create KDL Tree from URDF");
    return false;
  }

  ROS_INFO_NAMED(name_, "KDL tree loaded with %d joints and %d segments", 
                 kdl_tree.getNrOfJoints(), kdl_tree.getNrOfSegments());

  std::string base_frame, ee_frame;
  ctrl_nh.getParam("base_frame", base_frame);
  ctrl_nh.getParam("ee_frame", ee_frame);
  if(!kdl_tree.getChain(base_frame, ee_frame, kdl_chain_)) {
    ROS_ERROR_NAMED(name_, "Could not create KDL chain from base frame '%s' to end effector frame '%s'",
                    base_frame.c_str(), ee_frame.c_str());
    return false;
  }
  assert(num_jnts == kdl_chain_.getNrOfJoints());
  // should also assert that the joints are in the right order

  fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
  fk_vel_solver_.reset(new KDL::ChainFkSolverVel_recursive(kdl_chain_));

  jnt_states_cur_.q.resize(num_jnts);
  jnt_states_cur_.qdot.resize(num_jnts);
  qd_cmd_.resize(num_jnts);
  qd_cmd_last_.resize(num_jnts);
  first_update_ = true;

  current_state_    = State(num_jnts);
  desired_state_    = State(num_jnts);
  state_error_      = State(num_jnts);

  ros::NodeHandle gains_nh(ctrl_nh, "gains");
  gains_nh.getParam("lin_x_p", linear_cmd_gains_.m_floats[0]);
  gains_nh.getParam("lin_x_max", linear_cmd_maxs_.m_floats[0]);
  gains_nh.getParam("lin_y_p", linear_cmd_gains_.m_floats[1]);
  gains_nh.getParam("lin_y_max", linear_cmd_maxs_.m_floats[1]);
  gains_nh.getParam("lin_z_p", linear_cmd_gains_.m_floats[2]);
  gains_nh.getParam("lin_z_max", linear_cmd_maxs_.m_floats[2]);
  gains_nh.getParam("ang_x_p", angle_cmd_gains_.m_floats[0]);
  gains_nh.getParam("ang_x_max", angle_cmd_maxs_.m_floats[0]);
  gains_nh.getParam("ang_y_p", angle_cmd_gains_.m_floats[1]);
  gains_nh.getParam("ang_y_max", angle_cmd_maxs_.m_floats[1]);
  gains_nh.getParam("ang_z_p", angle_cmd_gains_.m_floats[2]);
  gains_nh.getParam("ang_z_max", angle_cmd_maxs_.m_floats[2]);

  double default_max_velocity = 1e9;
  ctrl_nh.getParam("max_velocity", default_max_velocity);
  double default_max_acceleration = 1e9;
  ctrl_nh.getParam("max_acceleration", default_max_acceleration);
  for (unsigned int i = 0; i < num_jnts; ++i) {
    qd_abs_maxs_[i] = default_max_velocity;
    qdd_abs_maxs_[i] = default_max_acceleration;

    ros::NodeHandle joint_nh(ctrl_nh, joint_names_[i]);
    joint_nh.getParam("max_velocity", qd_abs_maxs_[i]);
    joint_nh.getParam("max_acceleration", qdd_abs_maxs_[i]);
  }

  command_pose_sub_ = ctrl_nh.subscribe<geometry_msgs::PoseStamped>("command_pose", 1,
      boost::bind(&CartesianPositionControllerBase<State, HwIfaceAdapter, Controller>::commandPose,
                  this, _1));
  command_twist_sub_ = ctrl_nh.subscribe<geometry_msgs::TwistStamped>("command_twist", 1,
      boost::bind(&CartesianPositionControllerBase<State, HwIfaceAdapter, Controller>::commandTwist,
                  this, _1));

  // get inverse velocity solver parameters
  ros::NodeHandle ik_solver_vel_nh(ctrl_nh, "ik_solver_vel");
  double eps = 0.00001, max_iter = 150, alpha = 0.25;
  std::string solver_type = "pinv";
  ik_solver_vel_nh.getParam("eps", eps);
  ik_solver_vel_nh.getParam("max_iter", max_iter);
  ik_solver_vel_nh.getParam("alpha", alpha);
  ik_solver_vel_nh.getParam("solver_type", solver_type);
  if(solver_type == "pinv") {
    ROS_INFO_NAMED(name_, "KDL IK Velocity solver using basic pseudoinverse technique.");
    ik_vel_solver_.reset(new KDL::ChainIkSolverVel_pinv(kdl_chain_, eps, max_iter));
  }
  else if(solver_type == "pinv_givens") {
    ROS_INFO_NAMED(name_, "KDL IK Velocity solver using basic pseudoinverse - Givens variation.");
    ik_vel_solver_.reset(new KDL::ChainIkSolverVel_pinv_givens(kdl_chain_));
  }
  else if(solver_type == "wdls") {
    ROS_INFO_NAMED(name_, "KDL IK Velocity solver using weighted pseudoinverse.");
    ik_vel_solver_.reset(new KDL::ChainIkSolverVel_wdls(kdl_chain_, eps, max_iter));
    weight_js_sub_ = ctrl_nh.subscribe<std_msgs::Float64MultiArray>("weight_joint_space", 1,
        boost::bind(&CartesianPositionControllerBase<State, HwIfaceAdapter, Controller>::setWeightJS,
                    this, _1));
    weight_ts_sub_ = ctrl_nh.subscribe<std_msgs::Float64MultiArray>("weight_task_space", 1,
        boost::bind(&CartesianPositionControllerBase<State, HwIfaceAdapter, Controller>::setWeightTS,
                    this, _1));
  }
  else if(solver_type == "pinv_nso") {
    ROS_INFO_NAMED(name_, 
        "KDL IK Velocity solver using pseudoinverse with desired position optimization.");
    ik_vel_solver_.reset(new KDL::ChainIkSolverVel_pinv_nso(kdl_chain_, eps, max_iter, alpha));
    opt_pos_sub_ = ctrl_nh.subscribe<std_msgs::Float64MultiArray>("optimal_positions", 1,
        boost::bind(&CartesianPositionControllerBase<State, HwIfaceAdapter, Controller>::setOptPos, 
                    this, _1));
    weights_sub_ = ctrl_nh.subscribe<std_msgs::Float64MultiArray>("weights", 1,
        boost::bind(&CartesianPositionControllerBase<State, HwIfaceAdapter, Controller>::setWeights, 
                    this, _1));
  }
  return true;
}

template <class State, class HwIfaceAdapter, class Controller>
void CartesianPositionControllerBase<State, HwIfaceAdapter, Controller>::
starting(const ros::Time& time)
{
  first_update_ = true;
  // Hardware interface adapter
  hw_iface_adapter_.starting(time);
}

#define CLIP( x, xmin, xmax ) ( std::min( std::max( ( x ), ( xmin ) ), ( xmax ) ) )

template <class State, class HwIfaceAdapter, class Controller>
void CartesianPositionControllerBase<State, HwIfaceAdapter, Controller>::
update(const ros::Time& time, const ros::Duration& period)
{
  boost::lock_guard<boost::mutex> lock(mutex_lock_);
  int num_jnts = kdl_chain_.getNrOfJoints();
  double dt = period.toSec();

  for(int i = 0; i < num_jnts; i++) {
    // get the current joint positions/velocities
    jnt_states_cur_.q(i) = joint_states_[i].getPosition();
    jnt_states_cur_.qdot(i) = joint_states_[i].getVelocity();
  }

  // find the end effector Cartesian position and velocity
  fk_pos_solver_->JntToCart(jnt_states_cur_.q, x_cur_);
  fk_vel_solver_->JntToCart(jnt_states_cur_, xd_cur_frame_vel_);

  // convert position to tf vector
  tf::Vector3 x_cur_tf(x_cur_.p.x(), x_cur_.p.y(), x_cur_.p.z());
  // convert rotation to tf quaternion
  double quat_x, quat_y, quat_z, quat_w;
  x_cur_.M.GetQuaternion(quat_x, quat_y, quat_z, quat_w);
  tf::Quaternion quat_cur(quat_x, quat_y, quat_z, quat_w);

  // if this is the first update, initialize with the current pose
  if(first_update_) {
    x_des_ = x_cur_tf;
    quat_des_ = quat_cur;
    lin_fwd_vel_ = tf::Vector3();
    ang_fwd_vel_ = tf::Vector3();
    first_update_ = false;
  }

  ////////////////////// find linear velocity to command ////////////////////////////

  // calculate command as xdot = CLIP( Kp * (x_des - x_cur) )
  tf::Vector3 lin_vel_p_cmd = linear_cmd_gains_ * (x_des_ - x_cur_tf);
  for(int i = 0; i < 3; ++i)
    lin_vel_p_cmd.m_floats[i] = CLIP(lin_vel_p_cmd.m_floats[i], 
                                     -linear_cmd_maxs_[i], linear_cmd_maxs_[i]);
  tf::Vector3 lin_vel = lin_fwd_vel_ + lin_vel_p_cmd;
  ///////////////////////////////////////////////////////////////////////////////////
  
  ///////////////////// find angular velocity to command ////////////////////////////

  // get the axis/angle between the desired and current orientation
  if(quat_des_.dot(quat_cur) < 0.0)
    quat_cur *= -1.0; // make sure rotation takes the short way
  tf::Quaternion quat_between = quat_des_*quat_cur.inverse();
  tf::Vector3 axis_between = quat_between.getAxis();
  double angle_between = quat_between.getAngle(); // [-pi/2, pi/2]

  // calculate command as xdot = CLIP( Kp * theta * axis )
  tf::Vector3 ang_vel_p_cmd = angle_cmd_gains_ * angle_between * axis_between;
  for(int i = 0; i < 3; ++i)
    ang_vel_p_cmd.m_floats[i] = CLIP(ang_vel_p_cmd.m_floats[i], 
                                     -angle_cmd_maxs_[i], angle_cmd_maxs_[i]);
  tf::Vector3 ang_vel = ang_fwd_vel_ + ang_vel_p_cmd;
  ///////////////////////////////////////////////////////////////////////////////////

  // create twist command
  xd_cmd_(0) = lin_vel.x();
  xd_cmd_(1) = lin_vel.y();
  xd_cmd_(2) = lin_vel.z();
  xd_cmd_(3) = ang_vel.x();
  xd_cmd_(4) = ang_vel.y();
  xd_cmd_(5) = ang_vel.z();
  
  // find the joint velocity which achieves this Cartesian velocity
  ik_vel_solver_->CartToJnt(jnt_states_cur_.q, xd_cmd_, qd_cmd_);

  // scale the joint velocity so they are all below the maximums
  double max_max_ratio = 1.0;
  for(int i = 0; i < num_jnts; ++i) {
    // determine the bounds on the joint velocities
    double qd_delta = qdd_abs_maxs_[i] * dt; // acceleration bounds
    double qd_cmd_max = std::min(std::fabs(qd_cmd_last_(i)) + qd_delta, qd_abs_maxs_[i]);

    // find the ratio of joint command velocity to the maximum allowed
    max_max_ratio = std::max(max_max_ratio, std::fabs(qd_cmd_(i)) / qd_cmd_max);
  }
  double max_scale = 1.0 / max_max_ratio;
  for(int i = 0; i < num_jnts; ++i)
    qd_cmd_(i) *= max_scale;

  // update states
  for(int i = 0; i < num_jnts; ++i) {
    current_state_.position[i] = jnt_states_cur_.q(i);
    current_state_.velocity[i] = jnt_states_cur_.qdot(i);
    current_state_.acceleration[i] = 0.0;

    desired_state_.position[i] = jnt_states_cur_.q(i) + qd_cmd_(i)*dt;
    desired_state_.velocity[i] = qd_cmd_(i);
    desired_state_.acceleration[i] = (qd_cmd_(i) - qd_cmd_last_(i))/dt;

    state_error_.position[i] = desired_state_.position[i] - current_state_.position[i];
    state_error_.velocity[i] = desired_state_.velocity[i] - current_state_.velocity[i];
    state_error_.acceleration[i] = 0.0;

    qd_cmd_last_(i) = qd_cmd_(i);
  }

  // Hardware interface adapter: Generate and send commands
  hw_iface_adapter_.updateCommand(time, period, desired_state_, state_error_);
}

}

#endif
