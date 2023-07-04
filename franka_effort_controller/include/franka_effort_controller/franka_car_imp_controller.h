#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <cassert>
#include <stdexcept>

#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>

#include <franka_effort_controller/CarImpParamConfig.h>
#include <franka_effort_controller/WrenchConfig.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

// Boost
#include <boost/shared_ptr.hpp>
#include <boost/dynamic_bitset.hpp>

// ROS
#include <ros/node_handle.h>
#include <ros/package.h>
#include <ros/ros.h>
// #include <tf/transform_broadcaster.h>
// #include <tf/transform_listener.h>

// URDF
#include <urdf/model.h>

// actionlib
#include <actionlib/server/action_server.h>

// ROS messages
#include <control_msgs/JointTrajectoryControllerState.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/QueryTrajectoryState.h>
#include <trajectory_msgs/JointTrajectory.h>

// realtime_tools
#include <realtime_tools/realtime_box.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

// ros_controls
#include <realtime_tools/realtime_server_goal_handle.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/internal/demangle_symbol.h>

// Project
#include <trajectory_interface/trajectory_interface.h>

#include <joint_trajectory_controller/joint_trajectory_segment.h>
#include <joint_trajectory_controller/init_joint_trajectory.h>
// #include <joint_trajectory_controller/Frhardware_interface_adapter.h>
#include <joint_trajectory_controller/hold_trajectory_builder.h>
#include <joint_trajectory_controller/stop_trajectory_builder.h>

// Pluginlib
#include <pluginlib/class_list_macros.hpp>

// Project
#include <trajectory_interface/quintic_spline_segment.h>

// #include <joint_trajectory_controller/joint_trajectory_controller.h>  // some functions are in the inline namespace
// #include <franka_effort_controller/Frhardware_interface_adapter.h>
// #include <franka_effort_controller/rbdyn_wrapper.h>
#include <SpaceVecAlg/Conversions.h>
#include <franka_effort_controller/urdf.h>
#include <franka_effort_controller/pseudo_inversion.h>

// RBDyn Dynamics
#include <RBDyn/Body.h>
#include <RBDyn/Coriolis.h>
#include <RBDyn/FD.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>
#include <RBDyn/Jacobian.h>
#include <RBDyn/Joint.h>

#include <RBDyn/MultiBody.h>
#include <RBDyn/MultiBodyConfig.h>
#include <RBDyn/MultiBodyGraph.h>
#include <RBDyn/parsers/common.h>
// #include <mc_rbdyn_urdf/urdf.h>


namespace joint_trajectory_controller
{

template <class SegmentImpl>
class FrankaCarImpJointTrajectoryController : public controller_interface::MultiInterfaceController<
																								hardware_interface::EffortJointInterface,
																								franka_hw::FrankaModelInterface,
																								franka_hw::FrankaStateInterface
																								>
{
public:

  FrankaCarImpJointTrajectoryController();

  /** \name Non Real-Time Safe Functions
   *\{*/
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
  /*\}*/

  /** \name Real-Time Safe Functions
   *\{*/
  /** \brief Holds the current position. */
  void starting(const ros::Time& time);

  /** \brief Cancels the active action goal, if any. */
  void stopping(const ros::Time& /*time*/);

  void update(const ros::Time& time, const ros::Duration& period);
  /*\}*/

protected:

  struct TimeData
  {
    TimeData() : time(0.0), period(0.0), uptime(0.0) {}

    ros::Time     time;   ///< Time of last update cycle
    ros::Duration period; ///< Period of last update cycle
    ros::Time     uptime; ///< Controller uptime. Set to zero at every restart.
  };

  typedef actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>                  ActionServer;
  typedef std::shared_ptr<ActionServer>                                                       ActionServerPtr;
  typedef ActionServer::GoalHandle                                                            GoalHandle;
  typedef realtime_tools::RealtimeServerGoalHandle<control_msgs::FollowJointTrajectoryAction> RealtimeGoalHandle;
  typedef boost::shared_ptr<RealtimeGoalHandle>                                               RealtimeGoalHandlePtr;
  typedef trajectory_msgs::JointTrajectory::ConstPtr                                          JointTrajectoryConstPtr;
  typedef realtime_tools::RealtimePublisher<control_msgs::JointTrajectoryControllerState>     StatePublisher;
  typedef std::unique_ptr<StatePublisher>                                                     StatePublisherPtr;

  typedef JointTrajectorySegment<SegmentImpl> Segment;
  typedef std::vector<Segment> TrajectoryPerJoint;
  typedef std::vector<TrajectoryPerJoint> Trajectory;
  typedef std::shared_ptr<Trajectory> TrajectoryPtr;
  typedef std::shared_ptr<TrajectoryPerJoint> TrajectoryPerJointPtr;
  typedef realtime_tools::RealtimeBox<TrajectoryPtr> TrajectoryBox;
  typedef typename Segment::Scalar Scalar;

  // typedef FrHardwareInterfaceAdapter<hardware_interface::EffortJointInterface, typename Segment::State> HwIfaceAdapter;
  typedef typename hardware_interface::EffortJointInterface::ResourceHandleType JointHandle;

  bool                      verbose_;            ///< Hard coded verbose flag to help in debugging
  std::string               name_;               ///< Controller name.
  std::vector<JointHandle>  joint_handles_;      ///< Handles to controlled joints.
  std::vector<bool>         angle_wraparound_;   ///< Whether controlled joints wrap around or not.
  std::vector<std::string>  joint_names_;        ///< Controlled joint names.
  SegmentTolerances<Scalar> default_tolerances_; ///< Default trajectory segment tolerances.
  // HwIfaceAdapter            hw_iface_adapter_;   ///< Adapts desired trajectory state to HW interface.

  RealtimeGoalHandlePtr     rt_active_goal_;     ///< Currently active action goal, if any.

  /**
   * Thread-safe container with a smart pointer to trajectory currently being followed.
   * Can be either a hold trajectory or a trajectory received from a ROS message.
   *
   * We store the hold trajectory in a separate class member because the \p starting(time) method must be realtime-safe.
   * The (single segment) hold trajectory is preallocated at initialization time and its size is kept unchanged.
   */
  TrajectoryBox curr_trajectory_box_;
  TrajectoryPtr hold_trajectory_ptr_; ///< Last hold trajectory values.

  typename Segment::State current_state_;         ///< Preallocated workspace variable.
  typename Segment::State desired_state_;         ///< Preallocated workspace variable.
  typename Segment::State old_desired_state_;     ///< Preallocated workspace variable.
  typename Segment::State state_error_;           ///< Preallocated workspace variable.
  typename Segment::State desired_joint_state_;   ///< Preallocated workspace variable.
  typename Segment::State state_joint_error_;     ///< Preallocated workspace variable.
  Eigen::Matrix<double, 6, 1> pose_state_error;   // Cartesian pose state error

  std::unique_ptr<TrajectoryBuilder<SegmentImpl> > hold_traj_builder_;

  realtime_tools::RealtimeBuffer<TimeData> time_data_;
  TimeData old_time_data_;

  ros::Duration state_publisher_period_;
  ros::Duration action_monitor_period_;

  typename Segment::Time stop_trajectory_duration_;  ///< Duration for stop ramp. If zero, the controller stops at the actual position.
  boost::dynamic_bitset<> successful_joint_traj_;
  bool allow_partial_joints_goal_;

  // ROS API
  ros::NodeHandle    controller_nh_;
  ros::Subscriber    trajectory_command_sub_;
  ActionServerPtr    action_server_;
  ros::ServiceServer query_state_service_;
  StatePublisherPtr  state_publisher_;

  ros::Timer         goal_handle_timer_;
  ros::Time          last_state_publish_time_;

  virtual bool updateTrajectoryCommand(const JointTrajectoryConstPtr& msg, RealtimeGoalHandlePtr gh, std::string* error_string = nullptr);
  virtual void trajectoryCommandCB(const JointTrajectoryConstPtr& msg);
  virtual void goalCB(GoalHandle gh);
  virtual void cancelCB(GoalHandle gh);
  virtual void preemptActiveGoal();
  virtual bool queryStateService(control_msgs::QueryTrajectoryState::Request&  req,
                                 control_msgs::QueryTrajectoryState::Response& resp);

  /**
   * \brief Publish current controller state at a throttled frequency.
   * \note This method is realtime-safe and is meant to be called from \ref update, as it shares data with it without
   * any locking.
   */
  void publishState(const ros::Time& time);

  /**
   * \brief Hold the current position.
   *
   * Substitutes the current trajectory with a single-segment one going from the current position and velocity to
   * zero velocity.
   * \see parameter stop_trajectory_duration
   * \note This method is realtime-safe.
   */
  void setHoldPosition(const ros::Time& time, RealtimeGoalHandlePtr gh=RealtimeGoalHandlePtr());

  /**
   * @brief Returns the number of joints of the robot.
   */
  unsigned int getNumberOfJoints() const;

  /**
   * @brief Updates the states by sampling the specified trajectory for each joint
   * at the specified sampling time.
   *
   * The current state is updated based on the values transmitted by the
   * corresponding JointHandles.
   *
   * @param sample_time Time point at which the joint trajectories have to be sampled.
   * @param traj Trajectory containing all joint trajectories currently under execution.
   *
   * @note This function is NOT thread safe but intended to be used in the
   * update-function.
   */
  void updateStates(const ros::Time& sample_time, const Trajectory* const traj);

  /**
   * @brief Returns a trajectory consisting of joint trajectories with one pre-allocated
   * segment.
   */
  static TrajectoryPtr createHoldTrajectory(const unsigned int& number_of_joints);

private:
  /**
   * @brief Allows derived classes to perform additional checks
   * and to e.g. replace the newly calculated desired value before
   * the hardware interface is finally updated.
   *
   * @param curr_traj The trajectory that is executed during the current update.
   * @param time_data Updated time data.
   */
  virtual void updateFuncExtensionPoint(const Trajectory& curr_traj, const TimeData& time_data);

  /**
   * @brief Updates the pre-allocated feedback of the current active goal (if any)
   * based on the current state values.
   *
   * @note This function is NOT thread safe but intended to be used in the
   * update-function.
   */
  void setActionFeedback();

  // The Jacobian of RBDyn comes with orientation in the first three lines. Needs to be interchanged.
  const Eigen::VectorXi perm_indices_ =
    (Eigen::Matrix<int, 6, 1>() << 3, 4, 5, 0, 1, 2).finished(); //!< Permutation indices to switch position and orientation
  const Eigen::PermutationMatrix<Eigen::Dynamic, 6> jacobian_perm_ =
    Eigen::PermutationMatrix<Eigen::Dynamic, 6>(perm_indices_); //!< Permutation matrix to switch position and orientation entries
  const std::array<double, 16> d_F_T_EE_ = {1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
  const std::array<double, 16> d_EE_T_K_ = {1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};

  const Eigen::Matrix<double, 7, 7> cur_dx_filter1 = Eigen::MatrixXd::Identity(7, 7) * 0.99;
  const Eigen::Matrix<double, 7, 7> cur_dx_filter2 = Eigen::MatrixXd::Identity(7, 7) * 0.01;

  // typedef for brevity, if you need this more often: 
  // typedef Eigen::Matrix<double, 7, 1, Eigen::RowMajor> Mat43dR;

	//Franka Parameters sterm from CartesianImpedanceExampleController
  // Saturation
  Eigen::Matrix<double, 7, 1> saturateTorqueRate(
      const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
      const Eigen::Matrix<double, 7, 1>& tau_J_d);  // NOLINT (readability-identifier-naming)

  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  franka::RobotState franka_state_;

  std::vector<double> k_gains_;
  std::vector<double> d_gains_;

  // std::array<double, 7> tau_joint_;
  // std::array<double, 7> tau_d_joint_;
  // std::array<double, 7> tau_joint_ext_;
  // std::array<double, 6> tau_cart_ext_;

  double filter_params_{0.005};
  double nullspace_stiffness_{20.0};
  double nullspace_stiffness_target_{20.0};
  const double delta_tau_max_{1.0};
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_;
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_target_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_target_;
  Eigen::Matrix<double, 6, 6> cartesian_mass_;
  Eigen::Matrix<double, 6, 6> cartesian_mass_target_;
  double cartesian_damping_ratio_ = 1;

  Eigen::Matrix<double, 7, 1> des_q_;
  Eigen::Matrix<double, 7, 1> des_dq_;
  Eigen::Matrix<double, 7, 1> des_ddq_;
  Eigen::Matrix<double, 7, 1> cur_q_;
  Eigen::Matrix<double, 7, 1> cur_dq_before_filter_;
  Eigen::Matrix<double, 7, 1> cur_dq_;
  Eigen::Matrix<double, 7, 1> q_d_nullspace_;
  Eigen::Matrix<double, 7, 1> tau_d_; // command_torque
  Eigen::Matrix<double, 7, 1> tau_ext_;
   
 

  Eigen::Vector3d position_;
  Eigen::Quaterniond orientation_;
  Eigen::Vector3d position_d_;
  Eigen::Quaterniond orientation_d_;

  // std::mutex position_and_orientation_d_target_mutex_;
  // Eigen::Vector3d position_d_target_;
  // Eigen::Quaterniond orientation_d_target_;

  // RBDyn
  // rbdyn_wrapper rbdyn_wrapper_;   //!< Wrapper for RBDyn library for kinematics 
  /** multi body data */
  // rbd::MultiBody mb_;
  // rbd::MultiBodyConfig mbc_;
  // rbd::MultiBodyGraph mbg_;
  franka_mc_rbdyn_urdf::URDFParserResult rbdyn_urdf_;
  franka_mc_rbdyn_urdf::URDFParserResult des_rbdyn_urdf_ ;
  franka_mc_rbdyn_urdf::URDFParserResult cur_rbdyn_urdf_ ;

  // Eigen::MatrixXd RBDyn_M_;
  // Eigen::MatrixXd RBDyn_C_Mat_;
  // Eigen::MatrixXd jacobian_rbd ;
  // Eigen::MatrixXd jacobian_deviv_rbd;
  // Eigen::VectorXd RBDyn_C_;
  // Eigen::VectorXd RBDyn_G_;
  std::vector<size_t> rbd_indices_;

  // convert to Eigen. Get the current states
  Eigen::Matrix<double, 7, 1> coriolis_;
  Eigen::Matrix<double, 7, 1> gravity_;
  Eigen::Matrix<double, 6, 7> jacobian_;
  Eigen::Matrix<double, 7, 1> tau_J_d_;

  // full state of cartesian impedance control
  Eigen::Matrix<double, 6, 7> dot_jacobian_;
  Eigen::Matrix<double, 7, 7> mass_mat_;
  Eigen::Matrix<double, 6, 1> car_pos_error_;
  Eigen::Matrix<double, 6, 1> car_vel_error_;
  Eigen::Matrix<double, 6, 1> car_acc_;
  Eigen::Matrix<double, 6, 1> franka_external_wrench_;
  Eigen::Matrix<double, 6, 1> franka_external_wrench_initial_;
  Eigen::Matrix<double, 6, 1> external_wrench_;
  Eigen::Matrix<double, 6, 1> external_wrench_target_;
  Eigen::Matrix<double, 6, 6> external_wrench_transform_;
  bool applied_wrench_from_conf{false};

  // ros 
  ros::Publisher test_pub;

	// Dynamic reconfigure
  std::unique_ptr<dynamic_reconfigure::Server<franka_effort_controller::CarImpParamConfig>>
      dynamic_server_compliance_param_;
  std::unique_ptr<dynamic_reconfigure::Server<franka_effort_controller::WrenchConfig>>
      dynamic_server_wrench_param_;
  ros::NodeHandle dynamic_reconfigure_compliance_param_node_;
  void complianceParamCallback(franka_effort_controller::CarImpParamConfig& config,
                               uint32_t level);
  void dynamicWrenchCb(franka_effort_controller::WrenchConfig& config,
                               uint32_t level);

  // state update
  void getCurrentCartPose (Eigen::Vector3d& position,Eigen::Quaterniond& orientation);
  void getDesCartPose(Eigen::Vector3d& position_d,Eigen::Quaterniond& orientation_d);
  void getCartStatesFranka(Eigen::Vector3d& position, Eigen::Quaterniond& orientation,
    Eigen::Vector3d& position_d, Eigen::Quaterniond& orientation_d);
  void getCartStatesRBDyn(Eigen::Vector3d& position, Eigen::Quaterniond& orientation,
    Eigen::Vector3d& position_d, Eigen::Quaterniond& orientation_d); 
  void getCartFullStatesRBDyn(); 
  void getCartFullStates();
  //Test 
  int std_time_cnt{0};
  bool debug_flag_;

};

} // namespace

// #include <franka_effort_controller/franka_effort_controller_imp.h>
