#!/usr/bin/env python3

from __future__ import print_function
from asyncio import current_task

import copy
from re import S
import sys
from math import cos, dist, fabs, pi, tau


import geometry_msgs.msg
import moveit_commander
import moveit_msgs.msg
import rospy
from moveit_commander.conversions import pose_to_list
from moveit_msgs.msg import RobotTrajectory
from six.moves import input

from trajectory_msgs.msg import JointTrajectoryPoint



from transforms3d.quaternions import quat2mat, mat2quat

def all_close(goal, actual, tolerance):
	"""
	Convenience method for testing if the values in two lists are within a tolerance of each other.
	For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
	between the identical orientations q and -q is calculated correctly).
	@param: goal	   A list of floats, a Pose or a PoseStamped
	@param: actual	 A list of floats, a Pose or a PoseStamped
	@param: tolerance  A float
	@returns: bool
	"""
	if type(goal) is list:
		for index in range(len(goal)):
			if abs(actual[index] - goal[index]) > tolerance:
				return False

	elif type(goal) is geometry_msgs.msg.PoseStamped:
		return all_close(goal.pose, actual.pose, tolerance)

	elif type(goal) is geometry_msgs.msg.Pose:
		x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
		x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
		# Euclidean distance
		d = dist((x1, y1, z1), (x0, y0, z0))
		# phi = angle between orientations
		cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
		return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

	return True


class MoveGroupPanda(object):
	"""MoveGroupPanda"""

	def __init__(self):
		super(MoveGroupPanda, self).__init__()

		moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node("move_group_panda", anonymous=True)
  
		self.robot = moveit_commander.RobotCommander()

		self.scene = moveit_commander.PlanningSceneInterface()  


		group_name = "panda_arm"
		self.move_group = moveit_commander.MoveGroupCommander(group_name)    
		
		self.move_group.set_planning_pipeline_id("chomp")
		self.move_group.set_planner_id("CHMOP")
  
		# self.move_group.set_planning_pipeline_id("pilz_industrial_motion_planner")
		# self.move_group.set_planner_id("LIN")
		# self.move_group.set_planning_pipeline_id("ompl")
		# self.move_group.set_planner_id("RRTstar")
  
		## Create a `DisplayTrajectory`_ ROS publisher which is used to display
		## trajectories in Rviz:
		display_trajectory_publisher = rospy.Publisher(
			"/move_group/display_planned_path",
			moveit_msgs.msg.DisplayTrajectory,
			queue_size=20,
		)

		# We can get the name of the reference frame for this robot:
		planning_frame = self.move_group.get_planning_frame()
		print("============ Planning frame: %s" % planning_frame)

		# We can also print the name of the end-effector link for this group:
		# self.move_group.set_end_effector_link("probe_ee")
		self.move_group.set_end_effector_link("panda_link8")
		eef_link = self.move_group.get_end_effector_link()
		print("============ End effector link: %s" % eef_link)
		
		# We can get a list of all the groups in the robot:
		# group_names = robot.get_group_names()
		# print("============ Available Planning Groups:", robot.get_group_names())

		# pose_goal = geometry_msgs.msg.Pose()
		# link8
		# pose_goal.position.x = 0.306
		# pose_goal.position.y = 0.0
		# pose_goal.position.z = 0.55
  
  
		# probe_ee 0
		# pose_goal.position.x = 0.311
		# pose_goal.position.y = -0.006
		# pose_goal.position.z = 0.506
		# pose_goal.orientation.x = 1
		# pose_goal.orientation.y = 0
		# pose_goal.orientation.z = 0.0
		# pose_goal.orientation.w = 0.0

		# probe_ee 1
		# pose_goal.position.x = 0.325
		# pose_goal.position.y = -0.03
		# pose_goal.position.z = 0.354
		# pose_goal.orientation.x = -0.892376556791906
		# pose_goal.orientation.y = 0.4511411337048661
		# pose_goal.orientation.z = 0.008241541329095886
		# pose_goal.orientation.w = 0.008138632026473505

		point = [0.3244973091403415, -0.0026676111979773653, 0.5885575265056144, -0.9260554958644964, 0.3771234657570177, 0.010262797185633893, 0.009685423520767775]

		self.set_start_point(point)

		# Misc variables
		# self.robot = robot
		# self.scene = scene
		# self.move_group = move_group
		# self.display_trajectory_publisher = display_trajectory_publisher
		self.planning_frame = planning_frame
		self.eef_link = eef_link
		self.group_names = group_name
		# self.start_pose = pose_goal

	def go_to_joint_state(self):
		move_group = self.move_group

		joint_goal = move_group.get_current_joint_values()
		joint_goal[0] = 0
		joint_goal[1] = -tau / 8
		joint_goal[2] = 0
		joint_goal[3] = -tau / 4
		joint_goal[4] = 0
		joint_goal[5] = tau / 6  # 1/6 of a turn
		joint_goal[6] = 0

		# The go command can be called with joint values, poses, or without any
		# parameters if you have already set the pose or joint target for the group
		move_group.go(joint_goal, wait=True)

		# Calling ``stop()`` ensures that there is no residual movement
		move_group.stop()

		## END_SUB_TUTORIAL

		# For testing:
		current_joints = move_group.get_current_joint_values()
		return all_close(joint_goal, current_joints, 0.01)

	def go_to_pose(self):
		# Copy class variables to local variables to make the web tutorials more clear.
		# In practice, you should use the class variables directly unless you have a good
		# reason not to.
		move_group = self.move_group

		pose_goal = geometry_msgs.msg.Pose()
		pose_goal.orientation.w = 1.0
		pose_goal.position.x = 0.4
		pose_goal.position.y = 0.1
		pose_goal.position.z = 0.4


		move_group.set_pose_target(pose_goal)
		
		success = move_group.go(wait=True)
		move_group.stop()
		move_group.clear_pose_targets()

		current_pose = self.move_group.get_current_pose().pose
		return all_close(pose_goal, current_pose, 0.01)
    
	def go_to_command_arm_joint(self, joint1,joint2,joint3, joint4,joint5,joint6,joint7):
		## Planning to a Joint Goal
		move_group = self.move_group
		joint_goal = move_group.get_current_joint_values()
		joint_goal[0] = joint1
		joint_goal[1] = joint2
		joint_goal[2] = joint3
		joint_goal[3] = joint4
		joint_goal[4] = joint5
		joint_goal[5] = joint6
		joint_goal[6] = joint7

		move_group.go(joint_goal, wait=True)
		move_group.stop()
		current_joints = move_group.get_current_joint_values()
		print('current_joints',current_joints)
		print('joint_goal',joint_goal)
		return all_close(joint_goal, current_joints, 0.01)

	def execute_with_limit(self,plan,state):
		plan_temp=copy.deepcopy(plan)
		new_plan = self.move_group.retime_trajectory(self.move_group.get_current_state(),
                                          		plan_temp,
                                            	self.velocity_scale,
                                             	self.acceleration_scale,
												'iterative_time_parameterization')
												# 'iterative_spline_parameterization')
                                              	# 'time_optimal_trajectory_generation')
		# plan_temp=copy.deepcopy(plan)
		# new_plan= self.scale_trajectory_speed(plan_temp, self.line_velocity_scale)
		return self.move_group.execute(new_plan, wait=state)

	def set_start_point(self,point):
		pose_goal = geometry_msgs.msg.Pose()
		# probe_ee 1
		pose_goal.position.x = point[0]
		pose_goal.position.y = point[1]
		pose_goal.position.z = point[2]
		pose_goal.orientation.x = point[3]
		pose_goal.orientation.y = point[4]
		pose_goal.orientation.z = point[5]
		pose_goal.orientation.w = point[6]

		self.start_pose = pose_goal

	def move_to_start(self):
		# Copy class variables to local variables to make the web tutorials more clear.
		# In practice, you should use the class variables directly unless you have a good
		# reason not to.
		move_group = self.move_group
		pose_goal = self.start_pose
		print("pose_goal: ",pose_goal)

		waypoints = []
		wpose = self.move_group.get_current_pose().pose

		# waypoints.append(copy.deepcopy(wpose))
		waypoints.append(copy.deepcopy(pose_goal))

		(plan, fraction) = self.move_group.compute_cartesian_path(
										waypoints,   # waypoints to follow
										0.01,        # eef_step
										5)         # jump_threshold
		print("fraction: ", fraction)

		# plan_temp=copy.deepcopy(plan)
		# new_plan= self.scale_trajectory_speed(plan_temp, self.line_velocity_scale)
		return self.execute_with_limit(plan, True)    

		# move_group.set_pose_target(pose_goal)
		# success = move_group.go(wait=True)
		# move_group.stop()
		# move_group.clear_pose_targets()
		# current_pose = self.move_group.get_current_pose().pose
		# return all_close(pose_goal, current_pose, 0.01)

	def go_to_pose_goal(self,x,y,z,orientation_x,orientation_y,orientation_z,orientation_w):
		move_group = self.move_group
		pose_goal = geometry_msgs.msg.Pose()
		pose_goal.position.x = x
		pose_goal.position.y =y
		pose_goal.position.z = z

		pose_goal.orientation.x =orientation_x
		pose_goal.orientation.y = orientation_y
		pose_goal.orientation.z = orientation_z
		pose_goal.orientation.w = orientation_w

		
		move_group.set_pose_target(pose_goal)

		## Now, we call the planner to compute the plan and execute it.
		plan = move_group.go(wait=True)
		# Calling `stop()` ensures that there is no residual movement
		move_group.stop()
		# It is always good to clear your targets after planning with poses.
		# Note: there is no equivalent function for clear_joint_value_targets()
		move_group.clear_pose_targets()
		current_pose = self.move_group.get_current_pose().pose
		return all_close(pose_goal, current_pose, 0.01)

	def go_to_waypoints(self,waypoints,wait_bool):
		# move_group = self.move_group
		(plan, fraction) = self.move_group.compute_cartesian_path(
										waypoints,   # waypoints to follow
										0.01,        # eef_step
										5)         # jump_threshold
		print("fraction: ", fraction)
		if(fraction < 1):
			return False

		return self.execute_with_limit(plan, wait_bool)  


	def go_to_waypoints(self,waypoints,wait_bool,plan_loop_times):
		# move_group = self.move_group
		loop_times = 0
		fraction = 0
		while(fraction<1):
			(plan, fraction) = self.move_group.compute_cartesian_path(
											waypoints,   # waypoints to follow
											0.01,        # eef_step
											5)         # jump_threshold
			print("loop_times: ", loop_times)
			print("fraction: ", fraction)
			loop_times +=1
			if(loop_times ==plan_loop_times):
				return False

		return self.execute_with_limit(plan, wait_bool)  
		# plan_temp=copy.deepcopy(plan)
		# new_plan= self.scale_trajectory_speed(plan_temp, self.line_velocity_scale)
		# return move_group.execute(new_plan, wait=wait_bool)    

	def go_to_muti_line_movement(self,muti_point,state):
		# move_group = self.move_group
		waypoints = []
		muti_point_number=len(muti_point)
		wpose = self.move_group.get_current_pose().pose
		for  i in range(muti_point_number):
			wpose.position.x=muti_point[i][0]
			wpose.position.y=muti_point[i][1]
			wpose.position.z=muti_point[i][2]
			wpose.orientation.x=muti_point[i][3]
			wpose.orientation.y=muti_point[i][4]
			wpose.orientation.z=muti_point[i][5]
			wpose.orientation.w=muti_point[i][6]
			
			waypoints.append(copy.deepcopy(wpose))
		print("waypoints: ", waypoints)

		(plan, fraction) = self.move_group.compute_cartesian_path(
										waypoints,   # waypoints to follow
										0.01,        # eef_step
										5)         # jump_threshold
		print("fraction: ", fraction)
		return self.execute_with_limit(plan, state)     


	def set_joint_velocity_scale(self, velocity_scale):
		self.velocity_scale=velocity_scale
		self.move_group.set_max_velocity_scaling_factor(velocity_scale)

	def set_joint_acceleration_scale(self,acceleration_scale):
		self.acceleration_scale=acceleration_scale
		self.move_group.set_max_acceleration_scaling_factor(acceleration_scale)

	def set_line_velocity_scale(self, line_velocity_scale):
		self.line_velocity_scale=line_velocity_scale

	def scale_trajectory_speed(self,traj, scale):
		# Create a new trajectory object
		new_traj = RobotTrajectory()

		# Initialize the new trajectory to be the same as the input trajectory
		new_traj.joint_trajectory = traj.joint_trajectory

		# Get the number of joints involved
		n_joints = len(traj.joint_trajectory.joint_names)

		# Get the number of points on the trajectory
		n_points = len(traj.joint_trajectory.points)

		# Store the trajectory points
		points = list(traj.joint_trajectory.points)
			
		# Cycle through all points and joints and scale the time from start,
		# as well as joint speed and acceleration
		for i in range(n_points):
			point = JointTrajectoryPoint()

			# The joint positions are not scaled so pull them out first
			point.positions = traj.joint_trajectory.points[i].positions

			# Next, scale the time_from_start for this point
			point.time_from_start = traj.joint_trajectory.points[i].time_from_start / scale

			# Get the joint velocities for this point
			point.velocities = list(traj.joint_trajectory.points[i].velocities)

			# Get the joint accelerations for this point
			point.accelerations = list(traj.joint_trajectory.points[i].accelerations)

			# Scale the velocity and acceleration for each joint at this point
			for j in range(n_joints):
				point.velocities[j] = point.velocities[j] * scale
				point.accelerations[j] = point.accelerations[j] * scale * scale

			# Store the scaled trajectory point
			points[i] = point

		# Assign the modified points to the new trajectory
		new_traj.joint_trajectory.points = points

		# Return the new trajecotry
		return new_traj

	def set_trajectory_speed(self, traj, speed):
		# Create a new trajectory object
		new_traj = RobotTrajectory()
		
		# Initialize the new trajectory to be the same as the input trajectory
		new_traj.joint_trajectory = traj.joint_trajectory
		
		# Get the number of joints involved
		n_joints = len(traj.joint_trajectory.joint_names)
		
		# Get the number of points on the trajectory
		n_points = len(traj.joint_trajectory.points)
			
		# Store the trajectory points
		points = list(traj.joint_trajectory.points)
		
		# Cycle through all points and joints and scale the time from start,
		# as well as joint speed and acceleration
		for i in range(n_points):
			point = JointTrajectoryPoint()
			
			# The joint positions are not scaled so pull them out first
			point.positions = traj.joint_trajectory.points[i].positions

			# Next, scale the time_from_start for this point
			point.time_from_start = traj.joint_trajectory.points[i].time_from_start
			
			# Initialize the joint velocities for this point
			point.velocities = [speed] * n_joints
			
			# Get the joint accelerations for this point
			point.accelerations = [speed / 4.0] * n_joints
			
			# Store the scaled trajectory point
			points[i] = point

		# Assign the modified points to the new trajectory
		new_traj.joint_trajectory.points = points

		# Return the new trajecotry
		return new_traj

	def execute_plan(self, plan):
		# Copy class variables to local variables to make the web tutorials more clear.
		# In practice, you should use the class variables directly unless you have a good
		# reason not to.
		# move_group = self.move_group

		## BEGIN_SUB_TUTORIAL execute_plan
		##
		## Executing a Plan
		## ^^^^^^^^^^^^^^^^
		## Use execute if you would like the robot to follow
		## the plan that has already been computed:
		return self.execute_with_limit(plan, True)  

		## **Note:** The robot's current joint state must be within some tolerance of the
		## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
		## END_SUB_TUTORIAL

	def read_current_pose(self):
        	#get current pose
		#raw_input("stop0")
		end_effector_link = self.move_group.get_end_effector_link()
		#print("end_effector_link",end_effector_link)
		#raw_input("stop")
		x = self.move_group.get_current_pose(end_effector_link=end_effector_link).pose.position.x
		y = self.move_group.get_current_pose(end_effector_link=end_effector_link).pose.position.y
		z = self.move_group.get_current_pose(end_effector_link=end_effector_link).pose.position.z

		rx = self.move_group.get_current_pose(end_effector_link=end_effector_link).pose.orientation.x
		ry = self.move_group.get_current_pose(end_effector_link=end_effector_link).pose.orientation.y
		rz = self.move_group.get_current_pose(end_effector_link=end_effector_link).pose.orientation.z
		rw = self.move_group.get_current_pose(end_effector_link=end_effector_link).pose.orientation.w
		#print("start_pose",start_pose)
		
		current_pose=[x,y,z,rx,ry,rz,rw]
		print("current_pose",current_pose)
		return current_pose

	def read_current_joints(self):
    	#get current joints
		current_joints = self.move_group.get_current_joint_values()
		return current_joints

# class ArduinoPneumaticActuator(object):
    
# 	def __init__(self):
# 		super(ArduinoPneumaticActuator, self).__init__()
# 		# super().__init__(**kwargs)

# 		self.panda = MoveGroupPanda()

# 		self.sg_ratio = numpy.array([-9.7544,-13.3309,-9.2694,-6.2721,-11.7284,-9.8689])
# 		# self.sg_ratio = numpy.array([-10,-10,-10,-10,-10 ,-10])
		
# 		self.main_pwm_ = 0
# 		self.main_pressure_ = 0
# 		self.lip_pressure_ = 0
# 		self.strain_gauge_val_ = []
# 		self.main_value_limit_factor_ = 0.25
# 		self.main_valve_limit_ = 15  # negative pressure with positive value
# 		self.main_valve_trigger_pressure_ = 50 #self.main_valve_limit_*self.main_value_limit_factor_
# 		self.trajectory_state_ = 0
# 		self.ready_waypoints_ = []
# 		self.grasp_waypoints_ = []
# 		self.external_force = []
# 		self.base_sg_value = numpy.arange(6)
# 		self.sealing_force = numpy.arange(6)
# 		# self.ready_grasp_pose = self.panda.move_group.get_current_pose().pose
# 		# self.deep_grasp_pose = self.panda.move_group.get_current_pose().pose
# 		# self.quit_grasp_pose = self.panda.move_group.get_current_pose().pose

# 		self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
# 		roslaunch.configure_logging(self.uuid)
# 		self.launch_start_force = roslaunch.parent.ROSLaunchParent(self.uuid, ["/home/robot/a_panda_ws/src/franka_controllers/franka_effort_controller/launch/start_force_control.launch"])
# 		self.launch_stop_force = roslaunch.parent.ROSLaunchParent(self.uuid, ["/home/robot/a_panda_ws/src/franka_controllers/franka_effort_controller/launch/stop_force_control.launch"])

# 		self.mutex = threading.Lock()

# 		self.data_record_pub = rospy.Publisher("data_record_topic",Bool,queue_size=10)
# 		self.main_pwm_pub = rospy.Publisher("main_pressure_pwm",Int16,queue_size=10)
# 		self.main_mode_pub = rospy.Publisher("main_valve_switch",Int16,queue_size=10) # 0: off 1:succescive 2:internal
# 		self.main_interval_freq_pub = rospy.Publisher("main_interval_freq",Float32,queue_size=10)
# 		self.main_interval_duty_pub = rospy.Publisher("main_interval_duty_cycle",Float32,queue_size=10)
# 		self.main_valve_limit_pub = rospy.Publisher("main_valve_threshold",Int16,queue_size=10)
# 		self.main_pressure_value_sub = rospy.Subscriber("main_pressure_val",Float32,self.main_pressure_value_callback,queue_size=10)
# 		# self.panda_trajectory_result_sub = rospy.Subscriber("/franka_joint_trajectory_controller/follow_joint_trajectory/feedback", FollowJointTrajectoryActionFeedback,self.trajectory_state_callback,queue_size=10)
# 		self.panda_trajectory_result_sub = rospy.Subscriber("/franka_joint_trajectory_controller/follow_joint_trajectory/result", FollowJointTrajectoryActionResult,self.trajectory_state_callback,queue_size=10)
# 		# http://docs.ros.org/en/diamondback/api/control_msgs/html/msg/FollowJointTrajectoryActionResult.html 

# 		# 20230301 add lip_control and strain gauge
# 		self.lip_switch_pub = rospy.Publisher("lip_swtich",Int16,queue_size=10)  # low voltage active, 0: open; 255:close No velocity adjustment
# 		self.lip_limit_pressure_pub = rospy.Publisher("lip_pressure_sub",Float32,queue_size=10)  # Set lip pressure limit

# 		self.lip_pressure_value_sub = rospy.Subscriber("lip_pressure_val",Float32,self.lip_pressure_value_callback,queue_size=10) # Get lip pressure
# 		self.strain_gauge_sub = rospy.Subscriber("strain_gauge_values",Float32MultiArray,self.strain_gauge_callback,queue_size=10) # Get vertical force from strain gauge
# 		self.sg_sealing_force_pub = rospy.Publisher("sg_sealing_force",Float32MultiArray,queue_size=10)

# 		self.ext_force_sub = rospy.Subscriber("/franka_state_controller/F_ext",WrenchStamped,self.ext_force_callback,queue_size=10) # Get external force

# 	def turn_on_recording(self):
# 		msg = Bool()
# 		msg.data = True
# 		self.data_record_pub.publish(msg)

# 	def turn_off_recording(self):
# 		msg = Bool()
# 		msg.data = False
# 		self.data_record_pub.publish(msg)

# 	def trajectory_state_callback(self,data):
# 		self.trajectory_state_ = data.status.status
# 		# uint8 PENDING=0
# 		# uint8 ACTIVE=1
# 		# uint8 PREEMPTED=2
# 		# uint8 SUCCEEDED=3
# 		# uint8 ABORTED=4
# 		# uint8 REJECTED=5
# 		# uint8 PREEMPTING=6
# 		# uint8 RECALLING=7
# 		# uint8 RECALLED=8
# 		# uint8 LOST=9
# 	def main_pressure_value_callback(self,data):
# 		# print(-data.data * 7.5)
# 		self.main_pressure_ = int(-data.data * 7.5) # unit: mm/Hg
# 		# self.main_pressure_ = math.ceil(-data.data * 7.5)
# 	def lip_pressure_value_callback(self,data):
# 		# print(data.data)
# 		self.lip_pressure_ = data.data
# 	def strain_gauge_callback(self,data):
# 		# self.strain_gauge_val_ = numpy.array([data.data[0],data.data[1],data.data[2],data.data[3],data.data[4],data.data[5]])
# 		self.strain_gauge_val_ = numpy.array(data.data)

# 	def ext_force_callback(self,data):
# 		self.external_force = [data.wrench.force.x,data.wrench.force.y,data.wrench.force.z,data.wrench.torque.x,data.wrench.torque.y,data.wrench.torque.z]

# 	def turn_on_main_pressure(self,main_pwm,main_valve_limit):# main_valve_limit : positive with unit mm/Hg
# 		self.main_pwm_ = int(main_pwm)
# 		self.main_valve_limit_ = main_valve_limit
# 		self.main_valve_trigger_pressure_ = main_valve_limit*self.main_value_limit_factor_
  
# 		pwm_msg = Int16()
# 		pwm_msg.data = int(main_pwm)
# 		self.main_pwm_pub.publish(pwm_msg)

# 		valve_msg = Int16()
# 		valve_msg.data = main_valve_limit #-math.ceil(main_valve_limit/7.5)
# 		self.main_valve_limit_pub.publish(valve_msg)

# 		mode_msg = Int16()
# 		mode_msg.data = 1 
# 		self.main_mode_pub.publish(mode_msg)
# 		sleep(0.02)

# 	def turn_off_main_pressure(self):
# 		mode_msg = Int16()
# 		mode_msg.data = 0
# 		self.main_mode_pub.publish(mode_msg)
  
# 		self.main_pwm_ = 0
  
# 		pwm_msg = Int16()
# 		pwm_msg.data = self.main_pwm_
# 		self.main_pwm_pub.publish(pwm_msg)
# 		sleep(0.05)

# 	def interval_time(self,interval_time_data):
# 		# print("Interval time ", interval_time_data)
# 		sleep(interval_time_data)
		
# 	def set_main_limit_pressure(self,data):
# 		main_limit_pressure = Float32()
# 		main_limit_pressure.data = data
# 		self.main_valve_limit_pub.publish(main_limit_pressure)
# 		sleep(0.02)
# 	def set_lip_limit_pressure(self,data):
# 		lip_limit_pressure = Float32()
# 		lip_limit_pressure.data = data
# 		self.lip_limit_pressure_pub.publish(lip_limit_pressure)
# 		sleep(0.02)
# 	def turn_on_lip(self):
# 		lip_witch = Int16()
# 		lip_witch.data = 0
# 		self.lip_switch_pub(lip_witch)
# 		sleep(0.02)
# 	def turn_on_lip_with_limit_pressure(self,data):
# 		self.set_lip_limit_pressure(data)
# 		lip_witch = Int16()
# 		lip_witch.data = 0
# 		self.lip_switch_pub.publish(lip_witch)
# 		sleep(0.02)
# 	def turn_off_lip(self):
# 		lip_witch = Int16()
# 		lip_witch.data = 255
# 		self.lip_switch_pub.publish(lip_witch)
# 		sleep(0.05)
# 	def get_base_sg_value(self):
# 		self.base_sg_value = copy.deepcopy(self.strain_gauge_val_)
# 		# print("base_sg_value: ",self.base_sg_value)

# 	def get_sealing_force(self):
# 		print("strain_gauge_val_: ",self.strain_gauge_val_)
# 		print("base_sg_value: ",self.base_sg_value)
# 		# print("sg_ratio: ",self.sg_ratio)
# 		delta_sg_value = (self.strain_gauge_val_ - self.base_sg_value)
# 		self.sealing_force =self.sg_ratio * delta_sg_value
# 		# print("delta_sg_value: ",delta_sg_value)
# 		print("sealing_force: ",self.sealing_force)
# 		msg = Float32MultiArray()
# 		msg.data = [self.sealing_force[0],self.sealing_force[1],self.sealing_force[2],self.sealing_force[3],self.sealing_force[4],self.sealing_force[5]]
# 		self.sg_sealing_force_pub.publish(msg)

# 	def get_main_pressure_value(self):
# 		# print("self.main_pressure_ ",self.main_pressure_)
# 		# print("self.main_valve_trigger_pressure_ ",self.main_valve_trigger_pressure_)
# 		return self.main_pressure_

# 	def get_lip_pressure_value(self):
# 		# print("self.main_pressure_ ",self.main_pressure_)
# 		return self.lip_pressure_

# 	def get_strain_gauge_value(self):
# 		# print("self.main_pressure_ ",self.main_pressure_)
# 		return self.strain_gauge_val_

# 	def massage_once(self,main_pwm,main_valve_limit,interval_time_):
# 		self.turn_on_main_pressure(main_pwm,main_valve_limit)
# 		self.interval_time(interval_time_)
# 		self.turn_off_main_pressure()

# 	def transform_matrix_to_ros_pose(self,mat):
# 		"""
# 		Convert a transform matrix to a ROS pose.
# 		"""
# 		quat = mat2quat(mat[:3, :3])
# 		msg = Pose()
# 		msg.position = Point(x=mat[0, 3], y=mat[1, 3], z=mat[2, 3])
# 		msg.orientation = Quaternion(w=quat[0], x=quat[1], y=quat[2], z=quat[3])
# 		return msg


# 	def ros_pose_to_transform_matrix(self,msg):
# 		"""
# 		Convert a ROS pose to a transform matrix
# 		"""
# 		mat44 = numpy.eye(4)
# 		mat44[:3, :3] = quat2mat([msg.orientation.w, msg.orientation.x,
# 								msg.orientation.y, msg.orientation.z])
		
# 		mat44[0:3, -1] = [msg.position.x, msg.position.y, msg.position.z]
# 		return mat44

# 	def find_neighbor_pose(self,distance_z,pose):
# 		ready_grasp_pose = copy.deepcopy(pose)
# 		trans_matrix_cur_pose = self.ros_pose_to_transform_matrix(pose)
# 		trans_matrix_ready_pose = copy.deepcopy(trans_matrix_cur_pose)

# 		ready_grasp_distance =numpy.array([[0,0,distance_z,1]]).T 

# 		trans_matrix_ready_position = numpy.dot(trans_matrix_cur_pose,ready_grasp_distance)
# 		# print("trans_matrix_ready_position ", trans_matrix_ready_position)

# 		trans_matrix_ready_pose[0:3, 3] = trans_matrix_ready_position[0:3,0]
# 		# print("trans_matrix_ready_pose ", trans_matrix_ready_pose)

# 		ready_grasp_pose = self.transform_matrix_to_ros_pose(trans_matrix_ready_pose)
# 		# print("ready_grasp_pose ", ready_grasp_pose)
# 		return ready_grasp_pose


# 	def execute_massage(self,target_pose,interval_time_data):

# 		ready_grasp_pose1 = self.find_neighbor_pose(-0.03, target_pose)

# 		self.ready_waypoints_.append(copy.deepcopy(ready_grasp_pose1))
# 		# print(self.ready_waypoints_)
# 		self.trajectory_state_ = 1
# 		ready_pose_position_state = self.panda.go_to_waypoints(self.ready_waypoints_,True)

# 		self.ready_waypoints_ = []
# 		ready_grasp_pose2 = self.find_neighbor_pose(-0.01, target_pose)
# 		self.ready_waypoints_.append(copy.deepcopy(ready_grasp_pose2))

# 		if(not ready_pose_position_state):
# 			return False
# 		else:
# 			self.turn_on_main_pressure(250,250)

# 		deep_grasp_pose = self.find_neighbor_pose(0.02, target_pose)
# 		self.grasp_waypoints_ = []
# 		self.grasp_waypoints_.append(copy.deepcopy(deep_grasp_pose))
# 		# print(self.grasp_waypoints_)
# 		self.trajectory_state_ = 1
# 		trajectory_state = self.panda.go_to_waypoints(self.grasp_waypoints_,False)
# 		position_state = False
# 		pressure_state = False

# 		if(trajectory_state): # planning check
# 			print("self.trajectory_state_ ",self.trajectory_state_)	

# 			while(self.trajectory_state_ < 2): # trajectory running state
				
# 				if(self.get_main_pressure_value() >= self.main_valve_trigger_pressure_):
# 					print("!!!!!!!!!!!!Pressure Limit!")
# 					self.panda.move_group.stop()
# 					position_state = True
# 					pressure_state = True
# 					break
# 			if(self.trajectory_state_ == 3):
# 				position_state = True
# 				pressure_state = False
# 				print("Trajectory finished with no pressure!")
# 				# print("wpose",wpose)	
# 				# print("current_pose",current_pose)
# 			elif(self.trajectory_state_ == 4):
# 				print("Trajectory Aborted!")
# 				position_state = False
# 				pressure_state = False
# 				# return False
# 			elif(not self.trajectory_state_ == 1):
# 				print("Trajectory running problems. self.trajectory_state_ ",self.trajectory_state_)	
# 				# return False
				
# 			if(not pressure_state and position_state):
# 				print("!!!!!!!!!!!!Massage not working during trajectory!")
# 				pressure_cnt = 0
# 				if(self.get_main_pressure_value() < 20):
# 					pressure_state = False
# 					print("!!!!!!!!!!!!Low pressure. No hope for massage!")
# 					# return False

# 				else:
# 					pressure_meassure_flag = self.get_main_pressure_value() >= self.main_valve_trigger_pressure_
# 					while(not pressure_meassure_flag):
# 						if(pressure_cnt<50):
# 							self.interval_time(0.1)
# 							pressure_cnt =pressure_cnt+1
# 							pressure_state = self.get_main_pressure_value() >= self.main_valve_trigger_pressure_
# 							pressure_meassure_flag = pressure_state
# 						else:
# 							pressure_meassure_flag = True
# 							pressure_state = self.get_main_pressure_value() >= self.main_valve_trigger_pressure_
# 			if(pressure_state):
# 				print("!!!!!!!!!!!!Massage is working!")
# 				self.interval_time(interval_time_data)
# 				self.turn_off_main_pressure()
# 				return True
# 		else:
# 			print("!!!!!!!!!!!!trajectory planning failed!")
# 		self.turn_off_main_pressure()
# 		return False

# def main():

# 	pneumatic = ArduinoPneumaticActuator()
 
# 	wpose = pneumatic.panda.read_current_pose()
# 	# print("current_pose",wpose)
# 	pneumatic.panda.set_joint_velocity_scale(0.2)
# 	pneumatic.panda.set_joint_acceleration_scale(0.1)

# 	try:
# 		print("Press Ctrl-D to exit at any time")
# 		input(
# 			"============ 1 Press `Enter` to move to the start point ..."
# 		)
# 		pneumatic.panda.move_to_start()

# 		while " " != input("============ 2 Press `Enter` to repeat a trajectory ..."):
# 			waypoints = []
# 			wpose = pneumatic.panda.move_group.get_current_pose().pose
# 			wpose.position.x = 0.45
# 			wpose.position.y = 0
# 			wpose.position.z = 0.3501
# 			waypoints.append(copy.deepcopy(wpose))
			
# 			wpose.position.y = 0.2
# 			waypoints.append(copy.deepcopy(wpose))

# 			wpose.position.y = -0.201
# 			waypoints.append(copy.deepcopy(wpose))
			
# 			wpose.position.y = 0.2
# 			waypoints.append(copy.deepcopy(wpose))

# 			wpose.position.y = -0.201
# 			waypoints.append(copy.deepcopy(wpose))
			
# 			wpose.position.y = 0
# 			wpose.position.x = 0.3501
# 			waypoints.append(copy.deepcopy(wpose))

# 			wpose.position.x = 0.5501
# 			waypoints.append(copy.deepcopy(wpose))

# 			wpose.position.x = 0.35
# 			waypoints.append(copy.deepcopy(wpose))

# 			wpose.position.x = 0.55
# 			waypoints.append(copy.deepcopy(wpose))

# 			wpose.position.x = 0.45
# 			waypoints.append(copy.deepcopy(wpose))

# 			pneumatic.panda.go_to_waypoints(waypoints,True)

# 		input(
# 			"============ 3 Press `Enter` to move to the start point ..."
# 		)
# 		pneumatic.panda.move_to_start()
# 		print("============ Python tutorial demo complete!")
# 	except rospy.ROSInterruptException:
# 		return
# 	except KeyboardInterrupt:
# 		return



# if __name__ == "__main__":
# 	main()

