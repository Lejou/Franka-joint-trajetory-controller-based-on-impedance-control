///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2013, PAL Robotics S.L.
// Copyright (c) 2008, Willow Garage, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// * Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
// * Neither the name of PAL Robotics S.L. nor the names of its
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
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

// Pluginlib
#include <pluginlib/class_list_macros.hpp>

// Project
#include <trajectory_interface/quintic_spline_segment.h>
#include <franka_effort_controller/franka_compliance_controller.h>

/// \author Adolfo Rodriguez Tsouroukdissian, Stuart Glaser
/// \author Zihao Li 20221110


namespace joint_trajectory_controller
{
/*! \brief Calculates the orientation error between two quaternions
* 
* \param[in] orientation_d Reference orientation
* \param[in] orientation Current orientation
* \return Eigen Vector with the error
*/
Eigen::Vector3d calculateOrientationError(const Eigen::Quaterniond &orientation_d, Eigen::Quaterniond orientation)
{
  // Orientation error
  if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0)
  {
    orientation.coeffs() << -orientation.coeffs();
  }
  // "difference" quaternion
  const Eigen::Quaterniond error_quaternion(orientation * orientation_d.inverse());
  // convert to axis angle
  Eigen::AngleAxisd error_quaternion_angle_axis(error_quaternion);
  return error_quaternion_angle_axis.axis() * error_quaternion_angle_axis.angle();
}

inline void pseudoInverse(const Eigen::MatrixXd& M_, Eigen::MatrixXd& M_pinv_, bool damped = true) {
  double lambda_ = damped ? 0.2 : 0.0;

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(M_, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType sing_vals_ = svd.singularValues();
  Eigen::MatrixXd S_ = M_;  // copying the dimensions of M_, its content is not needed.
  S_.setZero();

  for (int i = 0; i < sing_vals_.size(); i++)
    S_(i, i) = (sing_vals_(i)) / (sing_vals_(i) * sing_vals_(i) + lambda_ * lambda_);

  M_pinv_ = Eigen::MatrixXd(svd.matrixV() * S_.transpose() * svd.matrixU().transpose());
}

template <class SegmentImpl>
inline void FrankaJointTrajectoryController<SegmentImpl>::starting(const ros::Time& time)
{
  // Update time data
  TimeData time_data;
  time_data.time   = time;
  time_data.uptime = ros::Time(0.0);
  time_data_.initRT(time_data);

  // Initialize the desired_state with the current state on startup
  for (unsigned int i = 0; i < getNumberOfJoints(); ++i)
  {
    desired_state_.position[i] = joint_handles_[i].getPosition();
    desired_state_.velocity[i] = joint_handles_[i].getVelocity();
  }

  // Hold current position
  setHoldPosition(time_data.uptime);

  // Initialize last state update time
  last_state_publish_time_ = time_data.uptime;

  // Hardware interface adapter
  // hw_iface_adapter_.starting(time_data.uptime);

  // Franka
  franka::RobotState initial_state = state_handle_->getRobotState();

  tau_joint_ = initial_state.tau_J;
  tau_d_joint_ = initial_state.dtau_J;
  tau_joint_ext_ = initial_state.tau_ext_hat_filtered;
  tau_cart_ext_ = initial_state.K_F_ext_hat_K;
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(initial_state.tau_J_d.data());
  tau_d_ = tau_J_d;

  // get jacobian
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kFlange);
  // convert to eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

  // set equilibrium point to current state
  position_d_ = initial_transform.translation();
  orientation_d_ = Eigen::Quaterniond(initial_transform.rotation());

  // set nullspace equilibrium configuration to initial q
  q_d_nullspace_ = q_initial;


}

template <class SegmentImpl>
inline void FrankaJointTrajectoryController<SegmentImpl>::stopping(const ros::Time& /*time*/)
{
  preemptActiveGoal();
}

template <class SegmentImpl>
inline void FrankaJointTrajectoryController<SegmentImpl>::trajectoryCommandCB(const JointTrajectoryConstPtr& msg)
{
  const bool update_ok = updateTrajectoryCommand(msg, RealtimeGoalHandlePtr());
  if (update_ok) {preemptActiveGoal();}
}

template <class SegmentImpl>
inline void FrankaJointTrajectoryController<SegmentImpl>::preemptActiveGoal()
{
  RealtimeGoalHandlePtr current_active_goal(rt_active_goal_);

  // Cancels the currently active goal
  if (current_active_goal)
  {
    // Marks the current goal as canceled
    rt_active_goal_.reset();
    current_active_goal->gh_.setCanceled();
  }
}

template <class SegmentImpl>
FrankaJointTrajectoryController<SegmentImpl>::FrankaJointTrajectoryController()
  : verbose_(false) // Set to true during debugging
{
  // The verbose parameter is for advanced use as it breaks real-time safety
  // by enabling ROS logging services
  if (verbose_)
  {
    ROS_WARN_STREAM(
        "The joint_trajectory_controller verbose flag is enabled. "
        << "This flag breaks real-time safety and should only be "
        << "used for debugging");
  }
}

template <class SegmentImpl>
bool FrankaJointTrajectoryController<SegmentImpl>::init(hardware_interface::RobotHW* robot_hw,
                                                  ros::NodeHandle&   root_nh,
                                                  ros::NodeHandle&   controller_nh)
{
  using namespace internal;

  // Cache controller node handle
  controller_nh_ = controller_nh;

  // Controller name
  name_ = getLeafNamespace(controller_nh_);

  // State publish rate
  double state_publish_rate = 50.0;
  controller_nh_.getParam("state_publish_rate", state_publish_rate);
  ROS_DEBUG_STREAM_NAMED(name_, "Controller state will be published at " << state_publish_rate << "Hz.");
  state_publisher_period_ = ros::Duration(1.0 / state_publish_rate);

  // Action status checking update rate
  double action_monitor_rate = 20.0;
  controller_nh_.getParam("action_monitor_rate", action_monitor_rate);
  action_monitor_period_ = ros::Duration(1.0 / action_monitor_rate);
  ROS_DEBUG_STREAM_NAMED(name_, "Action status changes will be monitored at " << action_monitor_rate << "Hz.");

  // Stop trajectory duration
  stop_trajectory_duration_ = 0.0;
  controller_nh_.getParam("stop_trajectory_duration", stop_trajectory_duration_);
  ROS_DEBUG_STREAM_NAMED(name_, "Stop trajectory has a duration of " << stop_trajectory_duration_ << "s.");

  // Checking if partial trajectories are allowed
  controller_nh_.param<bool>("allow_partial_joints_goal", allow_partial_joints_goal_, false);
  if (allow_partial_joints_goal_)
  {
    ROS_DEBUG_NAMED(name_, "Goals with partial set of joints are allowed");
  }

  // List of controlled joints
  joint_names_ = getStrings(controller_nh_, "joints");
  if (joint_names_.empty()) {return false;}
  const unsigned int n_joints = joint_names_.size();

  // URDF joints
  urdf::ModelSharedPtr urdf = getUrdf(root_nh, "robot_description");
  if (!urdf) {return false;}

  std::vector<urdf::JointConstSharedPtr> urdf_joints = getUrdfJoints(*urdf, joint_names_);
  if (urdf_joints.empty()) {return false;}
  assert(n_joints == urdf_joints.size());

  // Initialize members
  joint_handles_.resize(n_joints);
  angle_wraparound_.resize(n_joints);
  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "FrankaJointTrajectoryController: Error getting effort joint interface from hardware");
    return false;
  }
  for (unsigned int i = 0; i < n_joints; ++i)
  {
    // Joint handle

    try {
      joint_handles_[i] = effort_joint_interface->getHandle(joint_names_[i]);
      // joint_handles_.push_back(effort_joint_interface->getHandle(joint_names_[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "FrankaJointTrajectoryController: Exception getting joint handles: " << ex.what());
      return false;
    }

    // try {joint_handles_[i] = robot_hw->getHandle(joint_names_[i]);}
    // catch (...)
    // {
    //   ROS_ERROR_STREAM_NAMED(name_, "Could not find joint '" << joint_names_[i] << "' in '" <<
    //                                 this->getHardwareInterfaceType() << "'.");
    //   return false;
    // }

    // Whether a joint is continuous (ie. has angle wraparound)
    angle_wraparound_[i] = urdf_joints[i]->type == urdf::Joint::CONTINUOUS;
    const std::string not_if = angle_wraparound_[i] ? "" : "non-";

    // ROS_DEBUG_STREAM_NAMED(name_, "Found " << not_if << "continuous joint '" << joint_names_[i] << "' in '" <<
    //                               this->getHardwareInterfaceType() << "'.");
  }

  assert(joint_handles_.size() == angle_wraparound_.size());
  // ROS_DEBUG_STREAM_NAMED(name_, "Initialized controller '" << name_ << "' with:" <<
  //                        "\n- Number of joints: " << getNumberOfJoints() <<
  //                        "\n- Hardware interface type: '" << this->getHardwareInterfaceType() << "'" <<
  //                        "\n- Trajectory segment type: '" << hardware_interface::internal::demangledTypeName<SegmentImpl>() << "'");

  // Default tolerances
  ros::NodeHandle tol_nh(controller_nh_, "constraints");
  default_tolerances_ = getSegmentTolerances<Scalar>(tol_nh, joint_names_);

  // Hardware interface adapter
  // hw_iface_adapter_.init(joint_handles_, controller_nh_);

  // ROS API: Subscribed topics
  trajectory_command_sub_ = controller_nh_.subscribe("command", 1, &FrankaJointTrajectoryController::trajectoryCommandCB, this);

  // ROS API: Published topics
  state_publisher_.reset(new StatePublisher(controller_nh_, "state", 1));

  // ROS API: Action interface
  action_server_.reset(
      new ActionServer(controller_nh_, "follow_joint_trajectory",
                       std::bind(&FrankaJointTrajectoryController::goalCB, this, std::placeholders::_1),
                       std::bind(&FrankaJointTrajectoryController::cancelCB, this, std::placeholders::_1), false));
  action_server_->start();

  // ROS API: Provided services
  query_state_service_ = controller_nh_.advertiseService("query_state",
                                                         &FrankaJointTrajectoryController::queryStateService,
                                                         this);

  // Preeallocate resources
  current_state_       = typename Segment::State(n_joints);
  old_desired_state_   = typename Segment::State(n_joints);
  desired_state_       = typename Segment::State(n_joints);
  state_error_         = typename Segment::State(n_joints);
  desired_joint_state_ = typename Segment::State(1);
  state_joint_error_   = typename Segment::State(1);

  // successful_joint_traj_ = boost::dynamic_bitset<>(getNumberOfJoints()); // for joints
  successful_joint_traj_ = boost::dynamic_bitset<>(6); // for cartesian

  hold_trajectory_ptr_ = createHoldTrajectory(n_joints);
  assert(joint_names_.size() == hold_trajectory_ptr_->size());

  if (stop_trajectory_duration_ == 0.0)
  {
    hold_traj_builder_ = std::unique_ptr<TrajectoryBuilder<SegmentImpl> >(new HoldTrajectoryBuilder<SegmentImpl,hardware_interface::EffortJointInterface>(joint_handles_));
  }
  else
  {
    hold_traj_builder_ = std::unique_ptr<TrajectoryBuilder<SegmentImpl> >(new StopTrajectoryBuilder<SegmentImpl>(stop_trajectory_duration_, desired_state_));
  }

  {
    state_publisher_->lock();
    state_publisher_->msg_.joint_names = joint_names_;
    state_publisher_->msg_.desired.positions.resize(n_joints);
    state_publisher_->msg_.desired.velocities.resize(n_joints);
    state_publisher_->msg_.desired.accelerations.resize(n_joints);
    state_publisher_->msg_.actual.positions.resize(n_joints);
    state_publisher_->msg_.actual.velocities.resize(n_joints);
    state_publisher_->msg_.error.positions.resize(n_joints);
    state_publisher_->msg_.error.velocities.resize(n_joints);
    state_publisher_->unlock();
  }

  // Franka init

  std::vector<double> cartesian_stiffness_vector;
  std::vector<double> cartesian_damping_vector;

  std::string arm_id;
  if (!controller_nh_.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("FrankaJointTrajectoryController: Could not read parameter arm_id");
    return false;
  }
  debug_flag_ = false;
  if (!controller_nh_.getParam("debug_flag", debug_flag_)) {
    ROS_WARN_STREAM("FrankaCarImpJointTrajectoryController: Could not read parameter debug_flag, use default false");
  }
  // Front
  // std::vector<std::string> joint_names;
  // if (!controller_nh_.getParam("joint_names", joint_names) || joint_names.size() != 7) {
  //   ROS_ERROR(
  //       "FrankaJointTrajectoryController: Invalid or no joint_names parameters provided, "
  //       "aborting controller init!");
  //   return false;
  // }

  // obtain the model handle
  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "FrankaJointTrajectoryController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "FrankaJointTrajectoryController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  // obtian the state handle
  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "FrankaJointTrajectoryController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "FrankaJointTrajectoryController: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }

  // obtain joint handle  front
  // auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  // if (effort_joint_interface == nullptr) {
  //   ROS_ERROR_STREAM(
  //       "FrankaJointTrajectoryController: Error getting effort joint interface from hardware");
  //   return false;
  // }
  // for (size_t i = 0; i < 7; ++i) {
  //   try {
  //     joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
  //   } catch (const hardware_interface::HardwareInterfaceException& ex) {
  //     ROS_ERROR_STREAM(
  //         "FrankaJointTrajectoryController: Exception getting joint handles: " << ex.what());
  //     return false;
  //   }
  // }

  if (!controller_nh_.getParam("k_gains", k_gains_) || k_gains_.size() != 7) {
    ROS_ERROR(
        "FrankaJointTrajectoryController:  Invalid or no k_gain parameters provided, aborting "
        "controller init!");
    return false;
  }

  if (!controller_nh_.getParam("d_gains", d_gains_) || d_gains_.size() != 7) {
    ROS_ERROR(
        "FrankaJointTrajectoryController:  Invalid or no d_gain parameters provided, aborting "
        "controller init!");
    return false;
  }
  dynamic_reconfigure_compliance_param_node_ =
      ros::NodeHandle(controller_nh_.getNamespace() + "/dynamic_reconfigure_compliance_param_node");

  dynamic_server_compliance_param_ = std::make_unique<
      dynamic_reconfigure::Server<franka_effort_controller::compliance_paramConfig>>(
        dynamic_reconfigure_compliance_param_node_);
  dynamic_server_compliance_param_->setCallback(
      boost::bind(&FrankaJointTrajectoryController::complianceParamCallback, this, _1, _2));

  position_.setZero();
  orientation_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  position_d_.setZero();
  orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  tau_d_.setZero();

  cartesian_stiffness_.setZero();
  cartesian_damping_.setZero();

  // // RBDyn Initialization
  // std::string urdf_string;
  // std::string robot_description_;
  // controller_nh_.param<std::string>("robot_description", robot_description_, "/robot_description");
  // while (!controller_nh_.getParam(robot_description_, urdf_string))
  // {
  //   ROS_INFO_ONCE("Waiting for robot description in parameter %s on the ROS param server.",
  //                 robot_description_.c_str());
  //   usleep(100000);
  // }
  // std::cout <<"rbd test" << std::endl;

  // rbdyn_wrapper_.init_rbdyn(urdf_string, "hand");

  std::string urdf_string,robot_description;
  controller_nh_.param<std::string>("robot_description", robot_description, "/robot_description");
  // controller_nh_.param<std::string>("robot_description", robot_description, "/robot_description_urdf");
  while (!controller_nh_.getParam(robot_description, urdf_string))
  {
    ROS_INFO_ONCE("Waiting for robot description in parameter %s on the ROS param server.",
                  robot_description.c_str());
    usleep(100000);
  }
  std::cout <<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1" << std::endl;
  rbdyn_urdf_ = franka_mc_rbdyn_urdf::rbdyn_from_urdf(urdf_string);

  rbd_indices_.clear();
  std::cout <<"nrJoints " << rbdyn_urdf_.mb.nrJoints() << std::endl;
  // rbd_indices_.push_back(0);
  for (size_t i = 0; i < rbdyn_urdf_.mb.nrJoints(); i++) {
      if (rbdyn_urdf_.mb.joint(i).type() != rbd::Joint::Fixed){
        rbd_indices_.push_back(i);
        std::cout <<"rbd_indices_  " << i << " " <<rbdyn_urdf_.mb.body(i).name()<< std::endl;
      }
  }
  rbd_indices_.push_back(16);
  std::cout <<"rbd_indices_ " << rbd_indices_.size() << ' ' << rbdyn_urdf_.mb.body(16).name() << std::endl;

// rbd_indices_  2 panda_link1
// rbd_indices_  4 panda_link2
// rbd_indices_  6 panda_link3
// rbd_indices_  8 panda_link4
// rbd_indices_  10 panda_link5
// rbd_indices_  12 panda_link6
// rbd_indices_  14 panda_link7
// rbd_indices_  20 panda_leftfinger
// rbd_indices_  21 panda_rightfinger

  // std::cout <<"rbd test" << std::endl;
  // try {
  //   // std::string robot_description;
  //   // controller_nh_.getParam("robot_description", robot_description);
  //   std::cout <<"rbd test0" << std::endl;
  //   rbd::parsers::ParserResult res = rbd::parsers::from_file("/home/robot/a_panda_ws/src/franka_ros-noetic-devel/franka_description/robots/panda/panda.urdf");
  //   std::cout <<"rbd test1" << std::endl;
  //   mb_ = res.mb;
  //   mbc_ = res.mbc;
  //   mbg_ = res.mbg;
  // } catch (std::exception& e) {
  //   std::cout << e.what() << std::endl;
  // }
  // mbc_.gravity = cc
  // rbd::forwardKinematics(mb_, mbc_);
  // rbd::forwardVelocity(mb_, mbc_);

  return true;
}


template <class SegmentImpl>
void FrankaJointTrajectoryController<SegmentImpl>::update(const ros::Time& time, const ros::Duration& period)
{
  ros::WallTime start_, end_;
  double execution_time;
  if(debug_flag_){
  // ros::Time t1 = ros::Time::now();
  // double t_base = t1.toNSec();
  // ROS_INFO_THROTTLE(0.5," Base time : %f", t_base);
  // ROS_INFO_THROTTLE(0.5," Base time sec: %d", t1.sec);
    
    start_ = ros::WallTime::now();
  }

  // Get currently followed trajectory
  TrajectoryPtr curr_traj_ptr;
  curr_trajectory_box_.get(curr_traj_ptr);
  Trajectory& curr_traj = *curr_traj_ptr;

  old_time_data_ = *(time_data_.readFromRT());

  // Update time data
  TimeData time_data;
  time_data.time   = time;                                     // Cache current time
  time_data.period = period;                                   // Cache current control period
  time_data.uptime = old_time_data_.uptime + period; // Update controller uptime
  time_data_.writeFromNonRT(time_data); // TODO: Grrr, we need a lock-free data structure here!

  updateStates(time_data.uptime, curr_traj_ptr.get());

	if(debug_flag_){
    // ros::Time t2 = ros::Time::now();
    // ROS_INFO_THROTTLE(0.5," Update States time : %f", t2.toNSec()-t_base);

    end_ = ros::WallTime::now();
    execution_time = (end_ - start_).toNSec() * 1e-6;
    ROS_INFO_STREAM("start exectution time (ms): " << execution_time);
	}

  // Get desired states

  
  des_q_ = Eigen::Map<Eigen::Matrix<double, 7, 1>>(desired_state_.position.data());
  des_dq_ = Eigen::Map<Eigen::Matrix<double, 7, 1>>(desired_state_.velocity.data());
  des_ddq_ = Eigen::Map<Eigen::Matrix<double, 7, 1>>(desired_state_.acceleration.data());
  cur_q_ = Eigen::Map<Eigen::Matrix<double, 7, 1>>(current_state_.position.data());
  cur_dq_ = Eigen::Map<Eigen::Matrix<double, 7, 1>>(current_state_.velocity.data());

  // //--------------------------------------- Update state error----------------------------------------------
  getCartFullStates();
  // getCartFullStatesRBDyn();


	if(debug_flag_){
    end_ = ros::WallTime::now();
    execution_time = (end_ - start_).toNSec() * 1e-6;
    ROS_INFO_STREAM("GetCartStatesFranka exectution time (ms): " << execution_time);
	}

  //--------------------------------------- Update check states----------------------------------------------

  for (unsigned int i = 0; i < 6; ++i)
  {
    typename TrajectoryPerJoint::const_iterator segment_it = sample(curr_traj[i], time_data.uptime.toSec(), desired_joint_state_);
    
    if (curr_traj[i].end() == segment_it)
    {
      // Non-realtime safe, but should never happen under normal operation
      ROS_ERROR_NAMED(name_,
                      "Unexpected error: No trajectory defined at current time. Please contact the package maintainer.");
      return;
    }
    //Check tolerances

    const RealtimeGoalHandlePtr rt_segment_goal = segment_it->getGoalHandle();
    if (rt_segment_goal && rt_segment_goal == rt_active_goal_)
    {
      // Check tolerances at the end
      if (segment_it == --curr_traj[i].end())
      {
        const SegmentTolerancesPerJoint<Scalar>& tolerances = segment_it->getTolerances();
        // Controller uptime
        const ros::Time uptime = time_data_.readFromRT()->uptime;

        bool inside_goal_tolerances;
        if(i<3)
          inside_goal_tolerances= car_pos_error_(i)<0.005 ? true:false;
        else
          inside_goal_tolerances= car_pos_error_(i)<0.034 ? true:false;


        if (inside_goal_tolerances)
        {
          successful_joint_traj_[i] = 1;
        }
        else if (uptime.toSec() < segment_it->endTime() + tolerances.goal_time_tolerance)
        {
          // Still have some time left to meet the goal state tolerances
        }
        else
        {

          rt_segment_goal->preallocated_result_->error_code = control_msgs::FollowJointTrajectoryResult::GOAL_TOLERANCE_VIOLATED;
          rt_segment_goal->preallocated_result_->error_string = " pose " + std::to_string(i) + " goal error " + std::to_string( car_pos_error_[i] );
          rt_segment_goal->setAborted(rt_segment_goal->preallocated_result_);
          // Force this to run before destroying rt_active_goal_ so results message is returned
          rt_active_goal_->runNonRealtime(ros::TimerEvent());
          rt_active_goal_.reset();
          successful_joint_traj_.reset();
        }
      }
    }
  }

  RealtimeGoalHandlePtr current_active_goal(rt_active_goal_);
  if (current_active_goal && successful_joint_traj_.count() == 6)
  {
    current_active_goal->preallocated_result_->error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
    current_active_goal->setSucceeded(current_active_goal->preallocated_result_);
    current_active_goal.reset(); // do not publish feedback
    rt_active_goal_.reset();
    successful_joint_traj_.reset();
  }
	if(debug_flag_){
    end_ = ros::WallTime::now();
    execution_time = (end_ - start_).toNSec() * 1e-6;
    ROS_INFO_STREAM("Update check states exectution time (ms): " << execution_time);
	}

  // ---------------------------------------------- End of States Check ------------------------------------------------

  //--------------------------------------- Commands Calculation ----------------------------------------------

  //--------------------------------------- Cartesian PD control with damping ratio = 1 ----------------------------------------------
  // compute control
  // allocate variables
  // Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7);
  Eigen::Matrix<double, 7, 1> tau_task, tau_nullspace;

  // pseudoinverse for nullspace handling
  // kinematic pseuoinverse
  Eigen::MatrixXd jacobian_transpose_pinv;
  pseudoInverse(jacobian_.transpose(), jacobian_transpose_pinv);

  // Cartesian PD control with damping ratio = 1
  tau_task << jacobian_.transpose() *
                  (-cartesian_stiffness_ * car_pos_error_ - cartesian_damping_ * (jacobian_ * ( cur_dq_ )));
                  // (-cartesian_stiffness_ * car_pos_error_ - cartesian_damping_ * (jacobian_ * (dq )));
  // nullspace PD control with damping ratio = 1
  q_d_nullspace_ = des_q_;

  tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                    jacobian_.transpose() * jacobian_transpose_pinv) *
                        (nullspace_stiffness_ * (q_d_nullspace_ - cur_q_) -
                        (2.0 * sqrt(nullspace_stiffness_)) * cur_dq_);
                      //  (nullspace_stiffness_ * (q_d_nullspace_ - q) -
                      //   (2.0 * sqrt(nullspace_stiffness_)) * dq);
  // Desired torque
  tau_d_ << tau_task + tau_nullspace + coriolis_;
  // tau_d_ << tau_task + coriolis_;

  //--------------------------------------- End of control ----------------------------------------------
  
  // Saturate torque rate to avoid discontinuities
  tau_d_ << saturateTorqueRate(tau_d_, tau_J_d_);



  for (size_t i = 0; i < 7; ++i) {
    // tau_d[i] = k_gains_[i] * state_error_.position_[i]+ d_gains_[i] *state_error_.velocity[i];
    joint_handles_[i].setCommand(tau_d_[i]);
  }
  // std::cout << "tau_J_d_ " << std::endl << tau_J_d_ <<" "<< std::endl;
  // std::cout << "tau_d_ " << std::endl << tau_d_ <<" "<< std::endl;


  setActionFeedback();

  publishState(time_data.uptime);

  // update parameters changed online either through dynamic reconfigure or through the interactive
  // target by filtering
  cartesian_stiffness_ =
      filter_params_ * cartesian_stiffness_target_ + (1.0 - filter_params_) * cartesian_stiffness_;
  cartesian_damping_ =
      filter_params_ * cartesian_damping_target_  + (1.0 - filter_params_) * cartesian_damping_;
  nullspace_stiffness_ =
      filter_params_ * nullspace_stiffness_target_  + (1.0 - filter_params_) * nullspace_stiffness_;
  external_wrench_ =
      filter_params_ * external_wrench_target_  + (1.0 - filter_params_) * external_wrench_;

  // std::cout << std::endl <<"position_ " << std::endl << position_test <<" "<< std::endl;
  // std::cout << std::endl <<"position_d_ " << std::endl << position_d_ <<" "<< std::endl;
  // // std::cout << std::endl <<"orientation " << std::endl << orientation_ <<" "<< std::endl;
  // // std::cout << std::endl <<"orientation_d_ " << std::endl << orientation_d_ <<" "<< std::endl;
  // std::cout << std::endl <<"error " << std::endl << error <<" "<< std::endl;

	if(debug_flag_){
    // t2 = ros::Time::now();
    // ROS_INFO_THROTTLE(0.5," setCommand time : %f", t2.toNSec()-t_base);
    end_ = ros::WallTime::now();
    execution_time = (end_ - start_).toNSec() * 1e-6;
    ROS_INFO_STREAM("SetCommand time exectution time (ms): " << execution_time);
	}


}

template <class SegmentImpl>
bool FrankaJointTrajectoryController<SegmentImpl>::updateTrajectoryCommand(const JointTrajectoryConstPtr& msg, RealtimeGoalHandlePtr gh, std::string* error_string)
{
  typedef InitJointTrajectoryOptions<Trajectory> Options;
  Options options;
  options.error_string              = error_string;
  std::string error_string_tmp;

  // Preconditions
  if (!this->isRunning())
  {
    error_string_tmp = "Can't accept new commands. Controller is not running.";
    ROS_ERROR_STREAM_NAMED(name_, error_string_tmp);
    options.setErrorString(error_string_tmp);
    return false;
  }

  if (!msg)
  {
    error_string_tmp = "Received null-pointer trajectory message, skipping.";
    ROS_WARN_STREAM_NAMED(name_, error_string_tmp);
    options.setErrorString(error_string_tmp);
    return false;
  }

  // Time data
  TimeData* time_data = time_data_.readFromRT(); // TODO: Grrr, we need a lock-free data structure here!

  // Time of the next update
  const ros::Time next_update_time = time_data->time + time_data->period;

  // Uptime of the next update
  ros::Time next_update_uptime = time_data->uptime + time_data->period;

  // Hold current position if trajectory is empty
  if (msg->points.empty())
  {
    setHoldPosition(time_data->uptime, gh);
    ROS_DEBUG_NAMED(name_, "Empty trajectory command, stopping.");
    return true;
  }

  // Trajectory initialization options
  TrajectoryPtr curr_traj_ptr;
  curr_trajectory_box_.get(curr_traj_ptr);

  options.other_time_base           = &next_update_uptime;
  options.current_trajectory        = curr_traj_ptr.get();
  options.joint_names               = &joint_names_;
  options.angle_wraparound          = &angle_wraparound_;
  options.rt_goal_handle            = gh;
  options.default_tolerances        = &default_tolerances_;
  options.allow_partial_joints_goal = allow_partial_joints_goal_;

  // Update currently executing trajectory
  try
  {
    TrajectoryPtr traj_ptr(new Trajectory);
    *traj_ptr = initJointTrajectory<Trajectory>(*msg, next_update_time, options);
    if (!traj_ptr->empty())
    {
      curr_trajectory_box_.set(traj_ptr);
    }
    else
    {
      return false;
    }
  }
  catch(const std::exception& ex)
  {
    ROS_ERROR_STREAM_NAMED(name_, ex.what());
    options.setErrorString(ex.what());
    return false;
  }
  catch(...)
  {
    error_string_tmp = "Unexpected exception caught when initializing trajectory from ROS message data.";
    ROS_ERROR_STREAM_NAMED(name_, error_string_tmp);
    options.setErrorString(error_string_tmp);
    return false;
  }

  return true;
}

template <class SegmentImpl>
void FrankaJointTrajectoryController<SegmentImpl>::goalCB(GoalHandle gh)
{
  ROS_DEBUG_STREAM_NAMED(name_,"Received new action goal");

  // Precondition: Running controller
  if (!this->isRunning())
  {
    ROS_ERROR_NAMED(name_, "Can't accept new action goals. Controller is not running.");
    control_msgs::FollowJointTrajectoryResult result;
    result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL; // TODO: Add better error status to msg?
    gh.setRejected(result);
    return;
  }

  // If partial joints goals are not allowed, goal should specify all controller joints
  if (!allow_partial_joints_goal_)
  {
    if (gh.getGoal()->trajectory.joint_names.size() != joint_names_.size())
    {
      ROS_ERROR_NAMED(name_, "Joints on incoming goal don't match the controller joints.");
      control_msgs::FollowJointTrajectoryResult result;
      result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS;
      gh.setRejected(result);
      return;
    }
  }

  // Goal should specify valid controller joints (they can be ordered differently). Reject if this is not the case
  using internal::mapping;
  std::vector<unsigned int> mapping_vector = mapping(gh.getGoal()->trajectory.joint_names, joint_names_);

  if (mapping_vector.empty())
  {
    ROS_ERROR_NAMED(name_, "Joints on incoming goal don't match the controller joints.");
    control_msgs::FollowJointTrajectoryResult result;
    result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS;
    gh.setRejected(result);
    return;
  }

  // Try to update new trajectory
  RealtimeGoalHandlePtr rt_goal(new RealtimeGoalHandle(gh));
  std::string error_string;
  const bool update_ok = updateTrajectoryCommand(internal::share_member(gh.getGoal(), gh.getGoal()->trajectory),
                                                 rt_goal,
                                                 &error_string);
  rt_goal->preallocated_feedback_->joint_names = joint_names_;

  if (update_ok)
  {
    // Accept new goal
    preemptActiveGoal();
    gh.setAccepted();
    rt_active_goal_ = rt_goal;

    // Setup goal status checking timer
    goal_handle_timer_ = controller_nh_.createTimer(action_monitor_period_,
                                                    &RealtimeGoalHandle::runNonRealtime,
                                                    rt_goal);
    goal_handle_timer_.start();
  }
  else
  {
    // Reject invalid goal
    control_msgs::FollowJointTrajectoryResult result;
    result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
    result.error_string = error_string;
    gh.setRejected(result);
  }
}

template <class SegmentImpl>
void FrankaJointTrajectoryController<SegmentImpl>::cancelCB(GoalHandle gh)
{
  RealtimeGoalHandlePtr current_active_goal(rt_active_goal_);

  // Check that cancel request refers to currently active goal (if any)
  if (current_active_goal && current_active_goal->gh_ == gh)
  {
    // Reset current goal
    rt_active_goal_.reset();

    // Controller uptime
    const ros::Time uptime = time_data_.readFromRT()->uptime;

    // Enter hold current position mode
    setHoldPosition(uptime);
    ROS_DEBUG_NAMED(name_, "Canceling active action goal because cancel callback recieved from actionlib.");

    // Mark the current goal as canceled
    current_active_goal->gh_.setCanceled();
  }
}

template <class SegmentImpl>
bool FrankaJointTrajectoryController<SegmentImpl>::
queryStateService(control_msgs::QueryTrajectoryState::Request&  req,
                  control_msgs::QueryTrajectoryState::Response& resp)
{
  // Preconditions
  if (!this->isRunning())
  {
    ROS_ERROR_NAMED(name_, "Can't sample trajectory. Controller is not running.");
    return false;
  }

  // Convert request time to internal monotonic representation
  TimeData* time_data = time_data_.readFromRT();
  const ros::Duration time_offset = req.time - time_data->time;
  const ros::Time sample_time = time_data->uptime + time_offset;

  // Sample trajectory at requested time
  TrajectoryPtr curr_traj_ptr;
  curr_trajectory_box_.get(curr_traj_ptr);
  Trajectory& curr_traj = *curr_traj_ptr;

  typename Segment::State response_point = typename Segment::State(joint_names_.size());

  for (unsigned int i = 0; i < getNumberOfJoints(); ++i)
  {
    typename Segment::State state;
    typename TrajectoryPerJoint::const_iterator segment_it = sample(curr_traj[i], sample_time.toSec(), state);
    if (curr_traj[i].end() == segment_it)
    {
      ROS_ERROR_STREAM_NAMED(name_, "Requested sample time precedes trajectory start time.");
      return false;
    }

    response_point.position[i]     = state.position[0];
    response_point.velocity[i]     = state.velocity[0];
    response_point.acceleration[i] = state.acceleration[0];
  }

  // Populate response
  resp.name         = joint_names_;
  resp.position     = response_point.position;
  resp.velocity     = response_point.velocity;
  resp.acceleration = response_point.acceleration;

  return true;
}

template <class SegmentImpl>
void FrankaJointTrajectoryController<SegmentImpl>::publishState(const ros::Time& time)
{
  // Check if it's time to publish
  if (!state_publisher_period_.isZero() && last_state_publish_time_ + state_publisher_period_ < time)
  {
    if (state_publisher_ && state_publisher_->trylock())
    {
      last_state_publish_time_ += state_publisher_period_;

      state_publisher_->msg_.header.stamp          = time_data_.readFromRT()->time;
      state_publisher_->msg_.desired.positions     = desired_state_.position;
      state_publisher_->msg_.desired.velocities    = desired_state_.velocity;
      state_publisher_->msg_.desired.accelerations = desired_state_.acceleration;
      state_publisher_->msg_.desired.time_from_start = ros::Duration(desired_state_.time_from_start);
      state_publisher_->msg_.actual.positions      = current_state_.position;
      state_publisher_->msg_.actual.velocities     = current_state_.velocity;
      state_publisher_->msg_.actual.time_from_start = ros::Duration(current_state_.time_from_start);
      state_publisher_->msg_.error.positions       = state_error_.position;
      state_publisher_->msg_.error.velocities      = state_error_.velocity;
      state_publisher_->msg_.error.time_from_start = ros::Duration(state_error_.time_from_start);

      state_publisher_->unlockAndPublish();
    }
  }
}

template <class SegmentImpl>
inline void FrankaJointTrajectoryController<SegmentImpl>::
setHoldPosition(const ros::Time& time, RealtimeGoalHandlePtr gh)
{
  hold_traj_builder_
      ->setStartTime(time.toSec())
      ->setGoalHandle(gh)
      ->buildTrajectory(hold_trajectory_ptr_.get());
  hold_traj_builder_->reset();
  curr_trajectory_box_.set(hold_trajectory_ptr_);
}

template <class SegmentImpl>
inline void FrankaJointTrajectoryController<SegmentImpl>::
updateFuncExtensionPoint(const Trajectory& curr_traj, const TimeData& time_data)
{
  // To be implemented by derived class
}

template <class SegmentImpl>
inline unsigned int FrankaJointTrajectoryController<SegmentImpl>::getNumberOfJoints() const
{
  return joint_handles_.size();
}

template <class SegmentImpl>
void FrankaJointTrajectoryController<SegmentImpl>::updateStates(const ros::Time& sample_time, const Trajectory* const traj)
{
  old_desired_state_ = desired_state_;

  for (unsigned int joint_index = 0; joint_index < getNumberOfJoints(); ++joint_index)
  {
    const auto segment = sample( (*traj)[joint_index], sample_time.toSec(), desired_joint_state_);

    current_state_.position[joint_index] = joint_handles_[joint_index].getPosition();
    current_state_.velocity[joint_index] = joint_handles_[joint_index].getVelocity();
    // There's no acceleration data available in a joint handle

    desired_state_.position[joint_index] = desired_joint_state_.position[0];
    desired_state_.velocity[joint_index] = desired_joint_state_.velocity[0];
    desired_state_.acceleration[joint_index] = desired_joint_state_.acceleration[0];

    state_error_.position[joint_index] = angles::shortest_angular_distance(current_state_.position[joint_index],desired_joint_state_.position[0]);
    state_error_.velocity[joint_index] = desired_joint_state_.velocity[0] - current_state_.velocity[joint_index];
    state_error_.acceleration[joint_index] = 0.0;



    if (joint_index == 0)
    {
      const auto time_from_start = segment->timeFromStart();
      current_state_.time_from_start = sample_time.toSec() - segment->startTime() + time_from_start;
      desired_state_.time_from_start = time_from_start;
      state_error_.time_from_start = desired_state_.time_from_start - current_state_.time_from_start;
    }
  }
}

template <class SegmentImpl>
void FrankaJointTrajectoryController<SegmentImpl>::setActionFeedback()
{
  RealtimeGoalHandlePtr current_active_goal(rt_active_goal_);
  if (!current_active_goal)
  {
    return;
  }

  current_active_goal->preallocated_feedback_->header.stamp          = time_data_.readFromRT()->time;
  current_active_goal->preallocated_feedback_->desired.positions     = desired_state_.position;
  current_active_goal->preallocated_feedback_->desired.velocities    = desired_state_.velocity;
  current_active_goal->preallocated_feedback_->desired.accelerations = desired_state_.acceleration;
  current_active_goal->preallocated_feedback_->desired.time_from_start = ros::Duration(desired_state_.time_from_start);
  current_active_goal->preallocated_feedback_->actual.positions      = current_state_.position;
  current_active_goal->preallocated_feedback_->actual.velocities     = current_state_.velocity;
  current_active_goal->preallocated_feedback_->actual.time_from_start = ros::Duration(current_state_.time_from_start);
  current_active_goal->preallocated_feedback_->error.positions       = state_error_.position;
  current_active_goal->preallocated_feedback_->error.velocities      = state_error_.velocity;
  current_active_goal->preallocated_feedback_->error.time_from_start = ros::Duration(state_error_.time_from_start);
  current_active_goal->setFeedback( current_active_goal->preallocated_feedback_ );

}

template <class SegmentImpl>
typename FrankaJointTrajectoryController<SegmentImpl>::TrajectoryPtr
FrankaJointTrajectoryController<SegmentImpl>::createHoldTrajectory(const unsigned int& number_of_joints)
{
  TrajectoryPtr hold_traj {new Trajectory()};

  typename Segment::State default_state       = typename Segment::State(number_of_joints);
  typename Segment::State default_joint_state = typename Segment::State(1);
  for (unsigned int i = 0; i < number_of_joints; ++i)
  {
    default_joint_state.position[0]= default_state.position[i];
    default_joint_state.velocity[0]= default_state.velocity[i];
    Segment hold_segment(0.0, default_joint_state, 0.0, default_joint_state);

    TrajectoryPerJoint joint_segment;
    joint_segment.resize(1, hold_segment);
    hold_traj->push_back(joint_segment);
  }

  return hold_traj;
}

// Franka sterm from FrankaJointTrajectoryController
template <class SegmentImpl>
Eigen::Matrix<double, 7, 1> FrankaJointTrajectoryController<SegmentImpl>::saturateTorqueRate(
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

template <class SegmentImpl>
void FrankaJointTrajectoryController<SegmentImpl>::complianceParamCallback(
    franka_effort_controller::compliance_paramConfig& config,
    uint32_t /*level*/) {
  cartesian_stiffness_target_.setIdentity();
  cartesian_stiffness_target_.topLeftCorner(3, 3)
      << config.translational_stiffness * Eigen::Matrix3d::Identity();
  cartesian_stiffness_target_.bottomRightCorner(3, 3)
      << config.rotational_stiffness * Eigen::Matrix3d::Identity();
  cartesian_damping_target_.setIdentity();
  // Damping ratio = 1
  cartesian_damping_target_.topLeftCorner(3, 3)
      << 2.0  * sqrt(config.translational_stiffness) * Eigen::Matrix3d::Identity();
  cartesian_damping_target_.bottomRightCorner(3, 3)
      << 2.0 * sqrt(config.rotational_stiffness) * Eigen::Matrix3d::Identity();
  nullspace_stiffness_target_ = config.nullspace_stiffness;
 }


template <class SegmentImpl>
void FrankaJointTrajectoryController<SegmentImpl>::getCartFullStatesRBDyn(){
  // Eigen::Map<Eigen::Matrix<double, 7, 1>> des_q(desired_state_.position.data());
  // Eigen::Map<Eigen::Matrix<double, 7, 1>> des_dq(desired_state_.velocity.data());
  // Eigen::Map<Eigen::Matrix<double, 7, 1>> des_ddq(desired_state_.acceleration.data());
  // Eigen::Map<Eigen::Matrix<double, 7, 1>> q(current_state_.position.data());
  // Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(current_state_.velocity.data());
  // Forward Kinematics
  franka_mc_rbdyn_urdf::URDFParserResult des_rbdyn_urdf = rbdyn_urdf_;
  franka_mc_rbdyn_urdf::URDFParserResult cur_rbdyn_urdf = rbdyn_urdf_;


  des_rbdyn_urdf.mbc.zero(des_rbdyn_urdf.mb);
  cur_rbdyn_urdf.mbc.zero(cur_rbdyn_urdf.mb);
  // des_rbdyn_urdf.mbc.gravity = Eigen::Vector3d(0, 0, -9.81);
  // cur_rbdyn_urdf.mbc.gravity = Eigen::Vector3d(0, 0, -9.81);
  for (unsigned int i = 0; i <getNumberOfJoints(); i++) {
    size_t rbd_index = rbd_indices_[i];
    des_rbdyn_urdf.mbc.q[rbd_index][0] = des_q_[i];
    des_rbdyn_urdf.mbc.alpha[rbd_index][0] = des_dq_[i];
    cur_rbdyn_urdf.mbc.q[rbd_index][0] = cur_q_[i];
    cur_rbdyn_urdf.mbc.alpha[rbd_index][0] = 0;
  }

  // Obtain the desired states
  rbd::forwardKinematics(des_rbdyn_urdf.mb, des_rbdyn_urdf.mbc);

  sva::PTransformd des_tf = des_rbdyn_urdf.mbc.bodyPosW[rbd_indices_[getNumberOfJoints()]];

  Eigen::Matrix4d des_eig_tf = sva::conversions::toHomogeneous(des_tf);
  position_d_ = des_eig_tf.col(3).head(3);
  Eigen::Matrix3d des_rot_mat = des_eig_tf.block(0, 0, 3, 3);
  orientation_d_ = Eigen::Quaterniond(des_rot_mat).normalized();

  // Obtain the desired Jacobian and des car acc
  // rbd::Jacobian des_jac(des_rbdyn_urdf.mb, des_rbdyn_urdf.mb.body(rbd_indices_[getNumberOfJoints()]).name());

  // Eigen::Matrix<double, 6, 7> jacobian_des_rbdyn = des_jac.jacobian(des_rbdyn_urdf.mb, des_rbdyn_urdf.mbc);
  // Eigen::Matrix<double, 6, 7> jacobian_dot_des_rbdyn = des_jac.jacobianDot(des_rbdyn_urdf.mb, des_rbdyn_urdf.mbc);

  // Eigen::Matrix<double, 6, 7> jacobian_des = jacobian_perm_* des_jac.jacobian(des_rbdyn_urdf.mb, des_rbdyn_urdf.mbc);
  // Eigen::Matrix<double, 6, 7> jacobian_dot_des = jacobian_perm_* des_jac.jacobianDot(des_rbdyn_urdf.mb, des_rbdyn_urdf.mbc);;

  // for(int i=0;i<3;i++){
  //   jacobian_des.row(i) = jacobian_des_rbdyn.row(i+3);
  //   jacobian_des.row(i+3) = jacobian_des_rbdyn.row(i);
  //   jacobian_dot_des.row(i) = jacobian_dot_des_rbdyn.row(i+3);
  //   jacobian_dot_des.row(i+3) = jacobian_dot_des_rbdyn.row(i);
  // }
  // car_acc_ = jacobian_des * des_ddq_ + jacobian_dot_des * des_dq_;
  
  // car_acc_ = jacobian_des * des_ddq_ + jacobian_dot_des * des_dq_;

  // Eigen::Matrix<double, 6, 1> car_vel_des = jacobian_des * des_dq_;



  // std::cout << "current " << std::endl;
  rbd::ForwardDynamics fd(cur_rbdyn_urdf.mb);
  // cur_rbdyn_urdf.mbc.gravity = Eigen::Vector3d(0, 0, -9.81);
  rbd::forwardKinematics(cur_rbdyn_urdf.mb, cur_rbdyn_urdf.mbc);
  rbd::forwardVelocity(cur_rbdyn_urdf.mb, cur_rbdyn_urdf.mbc);
  fd.computeC(cur_rbdyn_urdf.mb, cur_rbdyn_urdf.mbc);
  gravity_.col(0) = -fd.C(); 

  for (unsigned int i = 0; i <getNumberOfJoints(); i++) {
    size_t rbd_index = rbd_indices_[i];
    cur_rbdyn_urdf.mbc.alpha[rbd_index][0] = cur_dq_[i];
  }

  // Obtain the current states
  rbd::forwardKinematics(cur_rbdyn_urdf.mb, cur_rbdyn_urdf.mbc);
  rbd::forwardVelocity(cur_rbdyn_urdf.mb, cur_rbdyn_urdf.mbc);

  sva::PTransformd cur_tf = cur_rbdyn_urdf.mbc.bodyPosW[rbd_indices_[getNumberOfJoints()]];

  Eigen::Matrix4d cur_eig_tf = sva::conversions::toHomogeneous(cur_tf);
  position_ = cur_eig_tf.col(3).head(3);
  Eigen::Matrix3d cur_rot_mat = cur_eig_tf.block(0, 0, 3, 3);
  orientation_ = Eigen::Quaterniond(cur_rot_mat).normalized();

  // std::cout << "Mass Matrix " << std::endl;

  // *** Mass Matrix
  // fd.computeH(cur_rbdyn_urdf.mb, cur_rbdyn_urdf.mbc);
  // std::cout << "Mass Matrix mass_mat_" << std::endl;
  // mass_mat_ = fd.H();// different from the date obtained from Franka


  // *** CoriolisVector 
  // compute C = cor/cent + G
  fd.computeC(cur_rbdyn_urdf.mb, cur_rbdyn_urdf.mbc);
  // Eigen::Matrix<double, 7, 1> N;
  // N.col(0) = fd.C();
  coriolis_.col(0) = fd.C() + gravity_;

  // std::cout << "cur_jac " << std::endl;
  // Obtain the current Jacobian 
  rbd::Jacobian jac(cur_rbdyn_urdf.mb, cur_rbdyn_urdf.mb.body(rbd_indices_[getNumberOfJoints()]).name());
  // Eigen::Matrix<double, 6, 7> jacobian_cur, jacobian_dot_cur;
  // jacobian_cur = jac.jacobian(cur_rbdyn_urdf.mb, cur_rbdyn_urdf.mbc);
  // jacobian_dot_cur = jac.jacobianDot(cur_rbdyn_urdf.mb, cur_rbdyn_urdf.mbc);

  jacobian_ = jacobian_perm_* jac.jacobian(cur_rbdyn_urdf.mb, cur_rbdyn_urdf.mbc);
  // dot_jacobian_ = jacobian_perm_* jac.jacobian(cur_rbdyn_urdf.mb, cur_rbdyn_urdf.mbc);

  // for(int i=0;i<3;i++){
  //   jacobian_.row(i) = jacobian_cur.row(i+3);
  //   jacobian_.row(i+3) = jacobian_cur.row(i);
  //   dot_jacobian_.row(i) = jacobian_dot_cur.row(i+3);
  //   dot_jacobian_.row(i+3) = jacobian_dot_cur.row(i);
  // }

  // std::cout << "car_vel_error_ " << std::endl;

  // car_vel_error_ = car_vel_des - jacobian_ * cur_dq_;
  // car_vel_error_ =  jacobian_des * des_dq_ - jacobian_ * cur_dq_;

  tau_J_d_ = tau_d_;

  // position error
  car_pos_error_.head(3) << position_ - position_d_;

  // orientation error
  if (orientation_d_.coeffs().dot(orientation_.coeffs()) < 0.0) {
    orientation_.coeffs() << -orientation_.coeffs();
  }
  car_pos_error_.tail(3) << calculateOrientationError(orientation_d_, orientation_);
  // std::cout << "End " << std::endl;

  std::cout << std::endl <<"Rbdyn car_pos_error_ " << std::endl << car_pos_error_ <<" "<< std::endl;

  // std::cout << std::endl <<"Rbdyn car_acc_ " << std::endl << car_acc_ <<" "<< std::endl;
  // std::cout << std::endl <<"Rbdyn dq " << std::endl << dq <<" "<< std::endl;
  // std::cout << std::endl <<"Rbdyn car_vel_error_ " << std::endl << car_vel_error_ <<" "<< std::endl;
  // std::cout << std::endl <<"Rbdyn mass_mat_ " << std::endl << mass_mat_ <<" "<< std::endl;
  // std::cout << std::endl <<"Rbdyn orientation " << std::endl << car_pos_error_ <<" "<< std::endl;
  // std::cout << std::endl <<"Rbdyn tau_d_ " << std::endl << tau_d_ <<" "<< std::endl;
  std::cout << std::endl <<"Rbdyn jacobian " << std::endl << jacobian_ <<" "<< std::endl;
  std::cout << std::endl <<"Rbdyn coriolis " << std::endl << coriolis_ <<" "<< std::endl;
  // std::cout << std::endl <<"Rbdyn gravity " << std::endl << gravity_ <<" "<< std::endl;

  }


template <class SegmentImpl>
void FrankaJointTrajectoryController<SegmentImpl>::getCartFullStates(){

  franka::RobotState franka_state = state_handle_->getRobotState();

  Eigen::Affine3d transform(Eigen::Matrix4d::Map(franka_state.O_T_EE.data()));
  Eigen::Vector3d position_c(transform.translation());
  Eigen::Quaterniond orientation_c(transform.rotation());

  std::array<double, 16> d_F_T_EE = {1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
  std::array<double, 16> d_EE_T_K = {1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
  std::array<double, 7> des_j_array = {des_q_[0],des_q_[1],des_q_[2],des_q_[3],des_q_[4],des_q_[5],des_q_[6]};

  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis_c(model_handle_->getCoriolis().data());
  // Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity_c(model_handle_->getGravity().data());
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian_cur(model_handle_->getZeroJacobian(franka::Frame::kFlange).data());
  // Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d_c(franka_state.tau_J_d.data());
  // Eigen::Map<Eigen::Matrix<double, 7, 7>> mass_mat(model_handle_->getMass().data());
  // Eigen::Map<Eigen::Matrix<double, 6, 1>> external_wrench(franka_state.O_F_ext_hat_K.data());
  std::array<double, 16> d_tf = model_handle_->getPose(franka::Frame::kFlange,des_j_array,d_F_T_EE,d_EE_T_K);

  Eigen::Affine3d transform_d(Eigen::Matrix4d::Map(d_tf.data()));
  Eigen::Vector3d position_d(transform_d.translation());
  Eigen::Quaterniond orientation_d(transform_d.rotation());


  coriolis_ = coriolis_c; 
  jacobian_ = jacobian_cur;
  tau_J_d_ = tau_d_;

  // position error
  car_pos_error_.head(3) << position_c - position_d;

  // orientation error
  if (orientation_d_.coeffs().dot(orientation_.coeffs()) < 0.0) {
    orientation_.coeffs() << -orientation_.coeffs();
  }
  car_pos_error_.tail(3) << calculateOrientationError(orientation_d, orientation_c);

  std::cout << std::endl <<"Franka car_pos_error_ " << std::endl << car_pos_error_ <<" "<< std::endl;
  std::cout << std::endl <<"Franka jacobian " << std::endl << jacobian_ <<" "<< std::endl;
  std::cout << std::endl <<"Franka coriolis " << std::endl << coriolis_ <<" "<< std::endl;

  }
  


} // namespace

namespace franka_effort_controller
{
  /**
   * \brief Joint trajectory controller that represents trajectory segments as <b>quintic splines</b> and sends
   * commands to an \b effort interface.
   */
  typedef joint_trajectory_controller::FrankaJointTrajectoryController<trajectory_interface::QuinticSplineSegment<double>>
          FrankaJointTrajectoryController;
}



PLUGINLIB_EXPORT_CLASS(franka_effort_controller::FrankaJointTrajectoryController,   controller_interface::ControllerBase)