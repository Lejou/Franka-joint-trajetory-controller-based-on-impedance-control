#!/usr/bin/env python3

from __future__ import print_function
from asyncio import current_task

import copy
import math
from re import S
import sys
from math import cos, dist, fabs, pi, tau
from time import sleep
from typing import Tuple

import geometry_msgs.msg
import moveit_commander
import moveit_msgs.msg
import rospy
from moveit_commander.conversions import pose_to_list
from moveit_msgs.msg import RobotTrajectory
from six.moves import input
from std_msgs.msg import String
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryActionFeedback,FollowJointTrajectoryActionResult

import numpy

from geometry_msgs.msg import Quaternion, Pose, Point
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
		self.move_group.set_end_effector_link("probe_ee")
		eef_link = self.move_group.get_end_effector_link()
		print("============ End effector link: %s" % eef_link)
		
		# We can get a list of all the groups in the robot:
		# group_names = robot.get_group_names()
		# print("============ Available Planning Groups:", robot.get_group_names())

		pose_goal = geometry_msgs.msg.Pose()
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
		pose_goal.position.x = 0.536
		pose_goal.position.y = -0.0158
		pose_goal.position.z = 0.140
		pose_goal.orientation.x = 1.0
		pose_goal.orientation.y = 0.0
		pose_goal.orientation.z = 0.0
		pose_goal.orientation.w = 0.0

		# Misc variables
		# self.robot = robot
		# self.scene = scene
		# self.move_group = move_group
		# self.display_trajectory_publisher = display_trajectory_publisher
		self.planning_frame = planning_frame
		self.eef_link = eef_link
		self.group_names = group_name
		self.start_pose = pose_goal

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

	def move_to_start(self):
		# Copy class variables to local variables to make the web tutorials more clear.
		# In practice, you should use the class variables directly unless you have a good
		# reason not to.
		move_group = self.move_group
		pose_goal = self.start_pose
		print("pose_goal: ",pose_goal)

		waypoints = []
		wpose = move_group.get_current_pose().pose

		# waypoints.append(copy.deepcopy(wpose))
		waypoints.append(copy.deepcopy(pose_goal))

		(plan, fraction) = move_group.compute_cartesian_path(
										waypoints,   # waypoints to follow
										0.01,        # eef_step
										0.00)         # jump_threshold
		print("fraction: ", fraction)

		plan_temp=copy.deepcopy(plan)
		new_plan= self.scale_trajectory_speed(plan_temp, self.line_velocity_scale)
		move_group.execute(new_plan, wait=True)    

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
		move_group = self.move_group

		(plan, fraction) = move_group.compute_cartesian_path(
										waypoints,   # waypoints to follow
										0.01,        # eef_step
										0.00)         # jump_threshold
		print("fraction: ", fraction)
		if(fraction < 1):
			return False
		plan_temp=copy.deepcopy(plan)
		new_plan= self.scale_trajectory_speed(plan_temp, self.line_velocity_scale)
		return move_group.execute(new_plan, wait=wait_bool)    

	def go_to_muti_line_movement(self,muti_point):
		move_group = self.move_group
		waypoints = []
		muti_point_number=len(muti_point)
		wpose = move_group.get_current_pose().pose
		for  i in range(muti_point_number):
			wpose.position.x=muti_point[i][0]
			wpose.position.y=muti_point[i][1]
			wpose.position.z=muti_point[i][2]
			wpose.orientation.x=muti_point[i][3]
			wpose.orientation.y=muti_point[i][4]
			wpose.orientation.z=muti_point[i][5]
			wpose.orientation.w=muti_point[i][6]
			
			waypoints.append(copy.deepcopy(wpose))

		(plan, fraction) = move_group.compute_cartesian_path(
										waypoints,   # waypoints to follow
										0.01,        # eef_step
										0.00)         # jump_threshold
		move_group.execute(plan, wait=True)    

	def plan_cartesian_path(self, scale=1):
		# Copy class variables to local variables to make the web tutorials more clear.
		# In practice, you should use the class variables directly unless you have a good
		# reason not to.
		move_group = self.move_group

		## BEGIN_SUB_TUTORIAL plan_cartesian_path
		##
		## Cartesian Paths
		## ^^^^^^^^^^^^^^^
		## You can plan a Cartesian path directly by specifying a list of waypoints
		## for the end-effector to go through. If executing  interactively in a
		## Python shell, set scale = 1.0.
		##
		waypoints = []

		wpose = move_group.get_current_pose().pose
		wpose.position.z -= scale * 0.1  # First move up (z)
		wpose.position.y += scale * 0.2  # and sideways (y)
		waypoints.append(copy.deepcopy(wpose))

		wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
		waypoints.append(copy.deepcopy(wpose))

		wpose.position.y -= scale * 0.1  # Third move sideways (y)
		waypoints.append(copy.deepcopy(wpose))

		# We want the Cartesian path to be interpolated at a resolution of 1 cm
		# which is why we will specify 0.01 as the eef_step in Cartesian
		# translation.  We will disable the jump threshold by setting it to 0.0,
		# ignoring the check for infeasible jumps in joint space, which is sufficient
		# for this tutorial.
		(plan, fraction) = move_group.compute_cartesian_path(
			waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
		)  # jump_threshold

		# Note: We are just planning, not asking move_group to actually move the robot yet:
		return plan, fraction

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
		move_group = self.move_group

		## BEGIN_SUB_TUTORIAL execute_plan
		##
		## Executing a Plan
		## ^^^^^^^^^^^^^^^^
		## Use execute if you would like the robot to follow
		## the plan that has already been computed:
		move_group.execute(plan, wait=True)

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
		#print("current_pose",current_pose)
		return current_pose

	def read_current_joints(self):
    	#get current joints
		current_joints = self.move_group.get_current_joint_values()
		return current_joints

class ArduinoPneumaticActuator(object):
    
	def __init__(self):
		super(ArduinoPneumaticActuator, self).__init__()
		# super().__init__(**kwargs)

		self.panda = MoveGroupPanda()

		self.main_pwm_ = 0
		self.main_pressure_ = 0
		self.main_value_limit_factor_ = 0.25
		self.main_valve_limit_ = 15  # negative pressure with positive value
		self.main_valve_trigger_pressure_ = 50 #self.main_valve_limit_*self.main_value_limit_factor_
		self.trajectory_state_ = 0
		self.ready_waypoints_ = []
		self.grasp_waypoints_ = []
		# self.ready_grasp_pose = self.panda.move_group.get_current_pose().pose
		# self.deep_grasp_pose = self.panda.move_group.get_current_pose().pose
		# self.quit_grasp_pose = self.panda.move_group.get_current_pose().pose


		self.main_pwm_pub = rospy.Publisher("main_pressure_pwm",Int16,queue_size=10)
		self.main_mode_pub = rospy.Publisher("main_valve_switch",Int16,queue_size=10) # 0: off 1:succescive 2:internal
		self.main_interval_freq_pub = rospy.Publisher("main_interval_freq",Float32,queue_size=10)
		self.main_interval_duty_pub = rospy.Publisher("main_interval_duty_cycle",Float32,queue_size=10)
		self.main_valve_limit_pub = rospy.Publisher("main_valve_threshold",Int16,queue_size=10)
		self.main_pressure_value_sub = rospy.Subscriber("main_pressure_val",Float32,self.main_pressure_value_callback,queue_size=10)
		# self.panda_trajectory_result_sub = rospy.Subscriber("/franka_joint_trajectory_controller/follow_joint_trajectory/feedback", FollowJointTrajectoryActionFeedback,self.trajectory_state_callback,queue_size=10)
		self.panda_trajectory_result_sub = rospy.Subscriber("/franka_joint_trajectory_controller/follow_joint_trajectory/result", FollowJointTrajectoryActionResult,self.trajectory_state_callback,queue_size=10)
		# http://docs.ros.org/en/diamondback/api/control_msgs/html/msg/FollowJointTrajectoryActionResult.html 

	def trajectory_state_callback(self,data):
		self.trajectory_state_ = data.status.status
		# uint8 PENDING=0
		# uint8 ACTIVE=1
		# uint8 PREEMPTED=2
		# uint8 SUCCEEDED=3
		# uint8 ABORTED=4
		# uint8 REJECTED=5
		# uint8 PREEMPTING=6
		# uint8 RECALLING=7
		# uint8 RECALLED=8
		# uint8 LOST=9
	def main_pressure_value_callback(self,data):
		# print(-data.data * 7.5)
		self.main_pressure_ = int(-data.data * 7.5)
		# self.main_pressure_ = math.ceil(-data.data * 7.5)

	def turn_on_main_pressure(self,main_pwm,main_valve_limit):
		self.main_pwm_ = int(main_pwm)
		self.main_valve_limit_ = main_valve_limit
		self.main_valve_trigger_pressure_ = main_valve_limit*self.main_value_limit_factor_
  
		pwm_msg = Int16()
		pwm_msg.data = int(main_pwm)
		self.main_pwm_pub.publish(pwm_msg)

		valve_msg = Int16()
		valve_msg.data = -math.ceil(main_valve_limit/7.5)
		self.main_valve_limit_pub.publish(valve_msg)

		mode_msg = Int16()
		mode_msg.data = 1 
		self.main_mode_pub.publish(mode_msg)

	def turn_off_main_pressure(self):
		mode_msg = Int16()
		mode_msg.data = 0
		self.main_mode_pub.publish(mode_msg)
  
		self.main_pwm_ = 0
  
		pwm_msg = Int16()
		pwm_msg.data = self.main_pwm_
		self.main_pwm_pub.publish(pwm_msg)

	def interval_time(self,interval_time_data):
		# print("Interval time ", interval_time_data)
		sleep(interval_time_data)

	def get_main_pressure_value(self):
		# print("self.main_pressure_ ",self.main_pressure_)
		# print("self.main_valve_trigger_pressure_ ",self.main_valve_trigger_pressure_)
		return self.main_pressure_

	def massage_once(self,main_pwm,main_valve_limit,interval_time_):
		self.turn_on_main_pressure(main_pwm,main_valve_limit)
		self.interval_time(interval_time_)
		self.turn_off_main_pressure()

	def transform_matrix_to_ros_pose(self,mat):
		"""
		Convert a transform matrix to a ROS pose.
		"""
		quat = mat2quat(mat[:3, :3])
		msg = Pose()
		msg.position = Point(x=mat[0, 3], y=mat[1, 3], z=mat[2, 3])
		msg.orientation = Quaternion(w=quat[0], x=quat[1], y=quat[2], z=quat[3])
		return msg


	def ros_pose_to_transform_matrix(self,msg):
		"""
		Convert a ROS pose to a transform matrix
		"""
		mat44 = numpy.eye(4)
		mat44[:3, :3] = quat2mat([msg.orientation.w, msg.orientation.x,
								msg.orientation.y, msg.orientation.z])
		
		mat44[0:3, -1] = [msg.position.x, msg.position.y, msg.position.z]
		return mat44

	def find_neighbor_pose(self,distance_z,pose):
		ready_grasp_pose = copy.deepcopy(pose)
		trans_matrix_cur_pose = self.ros_pose_to_transform_matrix(pose)
		trans_matrix_ready_pose = copy.deepcopy(trans_matrix_cur_pose)

		ready_grasp_distance =numpy.array([[0,0,distance_z,1]]).T 

		trans_matrix_ready_position = numpy.dot(trans_matrix_cur_pose,ready_grasp_distance)
		# print("trans_matrix_ready_position ", trans_matrix_ready_position)

		trans_matrix_ready_pose[0:3, 3] = trans_matrix_ready_position[0:3,0]
		# print("trans_matrix_ready_pose ", trans_matrix_ready_pose)

		ready_grasp_pose = self.transform_matrix_to_ros_pose(trans_matrix_ready_pose)
		# print("ready_grasp_pose ", ready_grasp_pose)
		return ready_grasp_pose


	def excute_massage(self,target_pose,interval_time_data):

		ready_grasp_pose1 = self.find_neighbor_pose(-0.03, target_pose)

		self.ready_waypoints_.append(copy.deepcopy(ready_grasp_pose1))
		# print(self.ready_waypoints_)
		self.trajectory_state_ = 1
		ready_pose_position_state = self.panda.go_to_waypoints(self.ready_waypoints_,True)

		self.ready_waypoints_ = []
		ready_grasp_pose2 = self.find_neighbor_pose(-0.01, target_pose)
		self.ready_waypoints_.append(copy.deepcopy(ready_grasp_pose2))

		if(not ready_pose_position_state):
			return False
		else:
			self.turn_on_main_pressure(250,250)

		deep_grasp_pose = self.find_neighbor_pose(0.02, target_pose)
		self.grasp_waypoints_ = []
		self.grasp_waypoints_.append(copy.deepcopy(deep_grasp_pose))
		# print(self.grasp_waypoints_)
		self.trajectory_state_ = 1
		trajectory_state = self.panda.go_to_waypoints(self.grasp_waypoints_,False)
		position_state = False
		pressure_state = False

		if(trajectory_state): # planning check
			print("self.trajectory_state_ ",self.trajectory_state_)	

			while(self.trajectory_state_ < 2): # trajectory running state
				
				if(self.get_main_pressure_value() >= self.main_valve_trigger_pressure_):
					print("!!!!!!!!!!!!Pressure Limit!")
					self.panda.move_group.stop()
					position_state = True
					pressure_state = True
					break
			if(self.trajectory_state_ == 3):
				position_state = True
				pressure_state = False
				print("Trajectory finished with no pressure!")
				# print("wpose",wpose)	
				# print("current_pose",current_pose)
			elif(self.trajectory_state_ == 4):
				print("Trajectory Aborted!")
				position_state = False
				pressure_state = False
				# return False
			elif(not self.trajectory_state_ == 1):
				print("Trajectory running problems. self.trajectory_state_ ",self.trajectory_state_)	
				# return False
				
			if(not pressure_state and position_state):
				print("!!!!!!!!!!!!Massage not working during trajectory!")
				pressure_cnt = 0
				if(self.get_main_pressure_value() < 20):
					pressure_state = False
					print("!!!!!!!!!!!!Low pressure. No hope for massage!")
					# return False

				else:
					pressure_meassure_flag = self.get_main_pressure_value() >= self.main_valve_trigger_pressure_
					while(not pressure_meassure_flag):
						if(pressure_cnt<50):
							self.interval_time(0.1)
							pressure_cnt =pressure_cnt+1
							pressure_state = self.get_main_pressure_value() >= self.main_valve_trigger_pressure_
							pressure_meassure_flag = pressure_state
						else:
							pressure_meassure_flag = True
							pressure_state = self.get_main_pressure_value() >= self.main_valve_trigger_pressure_
			if(pressure_state):
				print("!!!!!!!!!!!!Massage is working!")
				self.interval_time(interval_time_data)
				self.turn_off_main_pressure()
				return True
		else:
			print("!!!!!!!!!!!!trajectory planning failed!")
		self.turn_off_main_pressure()
		return False

def main():

	pneumatic = ArduinoPneumaticActuator()

	target_waypoints_record=numpy.array([
	[0.5688366450772172,0.1388975470589785,0.008124187746190947,-0.9953302944169784,0.09511044998878097,0.01600588491784604,-0.003926699216773336],
	[0.5668553637742422,0.09970769928262566,0.010944302995683719,-0.9983264476778041,0.044705937055753,0.036483097618237724,-0.0038297058845070094],
	[0.5628106988252418,0.0643957548268359,0.01008143965815711,-0.9970618434598765,0.07094913334807842,0.02312002691470415,0.017302171800458233],
	[0.5584343863065794,0.03975904135594148,0.008389030936953312,-0.9980116087300758,0.04514393813820029,-0.003886987657752298,0.04381489491404444],
	# [0.5342048945462686,-0.0935746062678533,0.017659177687004923,-0.977285760325082,0.03387803206456927,-0.16678595639192542,-0.126282486353766],
	[0.5560940537599456,-0.08095594470447379,0.01972287361291508,-0.993867952898086,-0.005126473094461903,0.02976389542703186,-0.10636880183958031],
	[0.5476583552672097,-0.13054814914639581,0.025105007511234956,-0.9933218492011167,0.0031386203910483204,0.04802476009020684,-0.10485931231889928],
	[0.5464035078733146,-0.1839749262315287,0.031406535447762285,-0.9978752270936568,0.00012070462093954903,0.0235901823828163,-0.06073318597211108]
	])
	# print("target_waypoints_record ", target_waypoints_record)

	(row,col) = target_waypoints_record.shape
	wpose = pneumatic.panda.move_group.get_current_pose().pose
	target_waypoints = []
	interval_time_ =2.5

	for i in range(row):
		wpose.position.x = target_waypoints_record[i,0]
		wpose.position.y = target_waypoints_record[i,1]
		wpose.position.z = target_waypoints_record[i,2]
		wpose.orientation.x=target_waypoints_record[i,3]
		wpose.orientation.y=target_waypoints_record[i,4]
		wpose.orientation.z=target_waypoints_record[i,5]
		wpose.orientation.w=target_waypoints_record[i,6]
		target_waypoints.append(copy.deepcopy(wpose))

	row_i = 0
	test_row = row
	try:
		print("Start the massage!")
		input(
			"============ Press `Enter` to move to the start point ..."
		)
		# # pneumatic.panda.set_joint_velocity_scale(0.2)
		# # pneumatic.panda.set_joint_acceleration_scale(0.2)
		pneumatic.panda.set_line_velocity_scale(0.4)
		pneumatic.panda.move_to_start()

		# pneumatic.panda.set_line_velocity_scale(0.4)
		# cur_pose = pneumatic.panda.move_group.get_current_pose().pose
		# temp_start_pose = pneumatic.find_neighbor_pose(-0.03, cur_pose)

		# waypoints_ = []
		# waypoints_.append(copy.deepcopy(temp_start_pose))
		# trajectory_state = pneumatic.panda.go_to_waypoints(waypoints_,True)

		pneumatic.panda.set_line_velocity_scale(0.25)
		
		for row_i in range(test_row):
			# input(
			# 	"============ massage pose "+ str(row_i)
			# )
			# if(row_i == 0):
			# 	pneumatic.ready_waypoints_.append(copy.deepcopy(ready_grasp_pose1))
			# 	pneumatic.ready_grasp_pose = copy.deepcopy(target_waypoints[row_i])
			pneumatic.excute_massage(target_waypoints[row_i], interval_time_)

		# row_i = 3
		# pneumatic.excute_massage(target_waypoints[row_i], interval_time_)		
		# row_i = 4
		# pneumatic.excute_massage(target_waypoints[row_i], interval_time_)

		# input(
		# 	"============ Press `Enter` to back to the start point ..."
		# )
		# # pneumatic.panda.set_joint_velocity_scale(0.2)
		# # pneumatic.panda.set_joint_acceleration_scale(0.2)
		pneumatic.panda.set_line_velocity_scale(0.4)
		pneumatic.panda.move_to_start()

	except rospy.ROSInterruptException:
		return
	except KeyboardInterrupt:
		return


if __name__ == "__main__":
	main()

