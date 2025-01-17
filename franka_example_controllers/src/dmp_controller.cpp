// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/dmp_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka_example_controllers/pseudo_inversion.h>

namespace franka_example_controllers {

// TODO: fix hardcoding
void compute_nullspace(
  const std::vector<joint_limits_interface::JointLimits>& jlimits,
  const Eigen::Map<Eigen::Matrix<double, 7, 1>, 0>& q,
  const Eigen::Map<Eigen::Matrix<double, 7, 1>>& dq,
  Eigen::VectorXd& tau_nullspace){

  double q_mean;
  tau_nullspace.resize(7);
  const double w_q = .1;
  const double w_dq = .1;
  for (std::size_t i=0; i<7; i++)
  {
    std::cout<<q(i)<<" ";
    q_mean = jlimits.at(i).max_position-jlimits.at(i).min_position;
    tau_nullspace(i) = -w_q*(q(i)-q_mean/2)/q_mean-w_dq*dq(i);
    // tau_nullspace(i) = -w_q*(q(i)-q_mean/2)/q_mean;
  }
  std::cout<<std::endl;

};


bool DmpController::init(
  hardware_interface::RobotHW* robot_hw, // ros
  ros::NodeHandle& node_handle
){


  // ! Set the logger level to debug
  if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
      ros::console::notifyLoggerLevelsChanged();
  }

  std::vector<double> cartesian_stiffness_vector;
  std::vector<double> cartesian_damping_vector;

  sub_equilibrium_pose_ = node_handle.subscribe(
      "equilibrium_pose", 20, &DmpController::equilibriumPoseCallback, this,
      ros::TransportHints().reliable().tcpNoDelay()
  );

  sub_dmp_goal_ = node_handle.subscribe(
      "dmp_goal", 20, &DmpController::dmpGoalCallback, this,
      ros::TransportHints().reliable().tcpNoDelay()
  );

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("DmpController: Could not read parameter arm_id");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "DmpController: Invalid or no joint_names parameters provided, "
        "aborting controller init!");
    return false;
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "DmpController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "DmpController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "DmpController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "DmpController: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "DmpController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
      joint_limits_interface::JointLimits limits;
      const bool rosparam_limits_ok = joint_limits_interface::getJointLimits(
        joint_names[i], node_handle, limits
      );
      this->jlimits.push_back(limits);
      
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
        "DmpController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  dynamic_reconfigure_compliance_param_node_ =
    ros::NodeHandle(node_handle.getNamespace() + "/dynamic_reconfigure_compliance_param_node");

  dynamic_server_compliance_param_ = std::make_unique<
    dynamic_reconfigure::Server<franka_example_controllers::compliance_paramConfig>>(

    dynamic_reconfigure_compliance_param_node_);
  dynamic_server_compliance_param_->setCallback(
    boost::bind(&DmpController::complianceParamCallback, this, _1, _2));

  position_d_.setZero();
  orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  position_d_target_.setZero();
  orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;

  cartesian_stiffness_.setZero();
  cartesian_damping_.setZero();

  const std::string dpath = "/home/docker/catkin_ws/src/trajectories/my_stuff/dmpcpp";
  this->dmp_model.reset(
    new dmp::Ros3dDMP<dmp::KulviciusDMP>{
      node_handle, "/kulvicius_dmp"
    }
  );
  // this->dmp_model.reset(
  //   new dmp::Ros3dDMP<dmp::OnlineDMP>{
  //     node_handle, "/kulvicius_dmp"
  //   }
  // );
  this->S_dmp = std::vector<double>(3);
  this->G_dmp = std::vector<double>(3);
  bool succ;
  this->dmp_model->init(dpath+"/config");
  succ = this->dmp_model->load_weights(dpath+"/data/weights.dat");

  this->pub_ctl_log = node_handle.advertise<dmpcpp::LogStamped>("/ee_log", 100);

  return true;
}

void DmpController::starting(const ros::Time& /*time*/) {
  // compute initial velocity with jacobian and set x_attractor and q_d_nullspace
  // to initial configuration
  franka::RobotState initial_state = state_handle_->getRobotState();
  // get jacobian
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  // convert to eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

  // set equilibrium point to current state
  position_d_ = initial_transform.translation();
  position_d_dmp = initial_transform.translation();
  orientation_d_ = Eigen::Quaterniond(initial_transform.rotation());
  position_d_target_ = initial_transform.translation();
  orientation_d_target_ = Eigen::Quaterniond(initial_transform.rotation());

  // set nullspace equilibrium configuration to initial q
  q_d_nullspace_ = q_initial;
}

void DmpController::update(
  const ros::Time& /*time*/,
  const ros::Duration& /*period*/
) 
{
  // get state variables
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

  // convert to Eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(
    // NOLINT (readability-identifier-naming)
    robot_state.tau_J_d.data()
  );
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.rotation());

  // set control output
  // compute error to last desired pose (trigger phase stopping)
  Eigen::Matrix<double, 6, 1> error;
  error.head(3) << position - position_d_dmp;
  // this->dmp_model->set_error(error.head(3));
  // error.head(3) << position - position_d_;
  // ROS_INFO_STREAM(error.head(3).matrix());


  // check convergency
  this->convergency_error= -position;
  this->convergency_error(0)+=this->G_dmp.at(0);
  this->convergency_error(1)+=this->G_dmp.at(1);
  this->convergency_error(2)+=this->G_dmp.at(2);

  // ROS_INFO_STREAM(this->convergency_error);
  if (this->convergency_error.norm() > 0.01 && this->dmp_executing){
  // if (this->dmp_executing){
    this->dmp_model->step();
    // log positions
    position_d_dmp(0)=this->dmp_model->dmps.at(0).y;
    position_d_dmp(1)=this->dmp_model->dmps.at(1).y;
    position_d_dmp(2)=this->dmp_model->dmps.at(2).y;

    this->ctl_log.pt_d.x = position_d_dmp(0);
    this->ctl_log.pt_d.y = position_d_dmp(1);
    this->ctl_log.pt_d.z = position_d_dmp(2);
    this->ctl_log.pt_is.x = position(0);
    this->ctl_log.pt_is.y = position(1);
    this->ctl_log.pt_is.z = position(2);
    this->ctl_log.stamp = ros::Time().now();
    this->pub_ctl_log.publish(this->ctl_log);
  }

  // set new control output
  error.head(3) << position - position_d_dmp;
  // ROS_INFO_STREAM(error.head(3).matrix());

  // orientation error
  if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }
  // "difference" quaternion
  Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
  error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  // Transform to base frame
  error.tail(3) << -transform.rotation() * error.tail(3);

  // compute control
  // allocate variables
  Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7);

  // pseudoinverse for nullspace handling
  // kinematic pseuoinverse
  Eigen::MatrixXd jacobian_transpose_pinv;
  pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

  // Cartesian PD control with damping ratio = 1
  tau_task << jacobian.transpose() *(
    -cartesian_stiffness_ * error - cartesian_damping_ * (jacobian * dq)
  );
    
  // nullspace PD control with damping ratio = 1
  // tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
  //                   jacobian.transpose() * jacobian_transpose_pinv) *
  //                      (nullspace_stiffness_ * (q_d_nullspace_ - q) -
  //                       (2.0 * sqrt(nullspace_stiffness_)) * dq);

  compute_nullspace(this->jlimits, q, dq, tau_nullspace);
  // Desired torque
  tau_d << tau_task + tau_nullspace + coriolis;
  // tau_d << tau_task + coriolis;
  // tau_d << coriolis;
  // Saturate torque rate to avoid discontinuities
  tau_d << saturateTorqueRate(tau_d, tau_J_d);
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d(i));
  }

  // update parameters changed online either through dynamic reconfigure or through the interactive
  // target by filtering
  cartesian_stiffness_ =
      filter_params_ * cartesian_stiffness_target_ + (1.0 - filter_params_) * cartesian_stiffness_;
  cartesian_damping_ =
      filter_params_ * cartesian_damping_target_ + (1.0 - filter_params_) * cartesian_damping_;
  nullspace_stiffness_ =
      filter_params_ * nullspace_stiffness_target_ + (1.0 - filter_params_) * nullspace_stiffness_;
  std::lock_guard<std::mutex> position_d_target_mutex_lock(
      position_and_orientation_d_target_mutex_);
  position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
  orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);
}

Eigen::Matrix<double, 7, 1> DmpController::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] =
        tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  return tau_d_saturated;
}

void DmpController::complianceParamCallback(
    franka_example_controllers::compliance_paramConfig& config,
    uint32_t /*level*/) {
  cartesian_stiffness_target_.setIdentity();
  cartesian_stiffness_target_.topLeftCorner(3, 3)
      << config.translational_stiffness * Eigen::Matrix3d::Identity();
  cartesian_stiffness_target_.bottomRightCorner(3, 3)
      << config.rotational_stiffness * Eigen::Matrix3d::Identity();
  cartesian_damping_target_.setIdentity();
  // Damping ratio = 1
  cartesian_damping_target_.topLeftCorner(3, 3)
      << 2.0 * sqrt(config.translational_stiffness) * Eigen::Matrix3d::Identity();
  cartesian_damping_target_.bottomRightCorner(3, 3)
      << 2.0 * sqrt(config.rotational_stiffness) * Eigen::Matrix3d::Identity();
  nullspace_stiffness_target_ = config.nullspace_stiffness;
}

void DmpController::equilibriumPoseCallback(
  const geometry_msgs::PoseStampedConstPtr& msg
){
  std::lock_guard<std::mutex> position_d_target_mutex_lock(
      position_and_orientation_d_target_mutex_
  );
  position_d_target_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
  orientation_d_target_.coeffs() << 
    msg->pose.orientation.x, msg->pose.orientation.y,
    msg->pose.orientation.z, msg->pose.orientation.w;
  if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0) {
    orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
  }
}

void DmpController::dmpGoalCallback(
  const geometry_msgs::PoseStampedConstPtr& msg
){
  std::lock_guard<std::mutex> position_d_target_mutex_lock(
      position_and_orientation_d_target_mutex_
  );
  franka::RobotState robot_state = state_handle_->getRobotState();
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());

  this->S_dmp.at(0) = position(0);
  this->S_dmp.at(1) = position(1);
  this->S_dmp.at(2) = position(2);
  this->G_dmp.at(0) = msg->pose.position.x;
  this->G_dmp.at(1) = msg->pose.position.y;
  this->G_dmp.at(2) = msg->pose.position.z;
  this->dmp_model->gen_trajectory(
    this->S_dmp, this->G_dmp, this->T_dmp);
  this->dmp_model->pub_trj_gen();
  this->dmp_model->reset(
    this->S_dmp, this->G_dmp, this->T_dmp);
  this->dmp_executing = true;
}

}  // namespace franka_example_controllers


PLUGINLIB_EXPORT_CLASS(franka_example_controllers::DmpController,
                       controller_interface::ControllerBase)
