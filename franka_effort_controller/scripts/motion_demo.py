#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import math
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from time import sleep

def all_close(goal, actual, tolerance):
	"""
	Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
	@param: goal       A list of floats, a Pose or a PoseStamped
	@param: actual     A list of floats, a Pose or a PoseStamped
	@param: tolerance  A float
	@returns: bool
	"""
	all_equal = True

	if type(goal) is list:
		for index in range(len(goal)):
			if abs(actual[index] - goal[index]) > tolerance:
				return False

	elif type(goal) is geometry_msgs.msg.PoseStamped:
		return all_close(goal.pose, actual.pose, tolerance)

	elif type(goal) is geometry_msgs.msg.Pose:
		return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

	return True


class Motion(object):
	"""throbot Motion"""
	def __init__(self):
		super(Motion, self).__init__()

		moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node('motion', anonymous=True)

		robot = moveit_commander.RobotCommander()

		scene = moveit_commander.PlanningSceneInterface()

		group_name = "th_arm"
		move_group = moveit_commander.MoveGroupCommander(group_name)
		group_name = "hand"
		hand_group = moveit_commander.MoveGroupCommander(group_name)


		display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
													moveit_msgs.msg.DisplayTrajectory,
													queue_size=20)
		#move_group.set_goal_position_tolerance(0.01)
		#move_group.set_goal_orientation_tolerance(0.01)
		# set reference frame
		#print("ref",move_group.get_pose_reference_frame())
		reference_frame = 'link_base'
		move_group.set_pose_reference_frame(reference_frame)


		planning_frame = move_group.get_planning_frame()
		#print("============ Planning frame: %s" % planning_frame)

		# We can also print the name of the end-effector link for this group:
		eef_link = move_group.get_end_effector_link()
		#print("============ End effector link: %s" % eef_link)

		# We can get a list of all the groups in the robot:
		group_names = robot.get_group_names()
		#print("============ Available Planning Groups:", robot.get_group_names())

		# Sometimes for debugging it is useful to print the entire state of the
		# robot:
		#print("============ Printing robot state")
		#print(robot.get_current_state())


		self.robot = robot
		self.scene = scene
		self.move_group = move_group
		self.hand_group=hand_group
		self.display_trajectory_publisher = display_trajectory_publisher
		self.velocity_scale=0.1
		self.line_velocity_scale=0.1
		#self.planning_frame = planning_frame
		#self.eef_link = eef_link
		#self.group_names = group_names

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

	def go_to_command_hand_joint(self, joint1):
		hand_group = self.hand_group
		joint_goal = hand_group.get_current_joint_values()
		#print("joint_goal",joint_goal)
		joint_goal[0] = joint1
		hand_group.go(joint_goal, wait=True)
		hand_group.stop()
		return 0

	def go_to_hand_position(self, hand_position):
			"hand_position  is  gripper size"
			hand_group = self.hand_group
			joint1, grasp_depth_offset=self.gripper_dis_to_rad(hand_position)
			#print("joint_goal",joint_goal)
			#raw_input("stop")
			joint_goal = hand_group.get_current_joint_values()
			#print("joints",joints)
			print("joint_goal",joint_goal)
			joint_goal[0] = joint1
			joint_goal[1] = joint1
			joint_goal[2] = -joint1
			joint_goal[3] = joint1
			joint_goal[4] = joint1
			joint_goal[5] = -joint1
			hand_group.go(joint_goal, wait=True)
			hand_group.stop()

	def gripper_rad_to_dis(self, radian):
		# unit: radian:rad  other: m
		R = 0.07
		gripper_distance = 2*R*math.sin(radian-0.15)+0.025
		grasp_depth_offset = R*(1-math.cos(radian)) # From the tcp to top of the gripper finger
		return gripper_distance,grasp_depth_offset

	def gripper_dis_to_rad(self, dis):
		R = 0.07
		rad=math.asin((dis+0.025-0.05)/2/R)+0.15
		grasp_depth_offset = R*(1-math.cos(rad))
		return rad,grasp_depth_offset

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

	def set_trajectory_speed(traj, speed):
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

	def go_to_line_movement(self,x,y,z,orientation_x,orientation_y,orientation_z,orientation_w):
		move_group = self.move_group
		waypoints = []
		# print("get current pose",move_group.get_current_pose() )
		wpose = move_group.get_current_pose().pose
		#print('get_current_pose',wpose)
		wpose.position.x=x
		wpose.position.y=y
		wpose.position.z=z
		wpose.orientation.x=orientation_x
		wpose.orientation.y=orientation_y
		wpose.orientation.z=orientation_z
		wpose.orientation.w=orientation_w

		waypoints.append(copy.deepcopy(wpose))
		attempts=0
		while attempts<100:
			(plan, fraction) = move_group.compute_cartesian_path(
											waypoints,   # waypoints to follow
											0.01,        # eef_step
											0.0,          # jump_threshold, 0 stand for can't jump
											True)           # avoid_collisions
			if fraction==1.0:
				break
			attempts=attempts+1
		print('attempts',attempts)
		#print('plan1',plan)
		#print('plan1 add==========================',id(plan))
		plan_temp=copy.deepcopy(plan)
		#print('plan_temp add=========================',id(plan_temp))
		new_plan= self.scale_trajectory_speed(plan_temp, self.line_velocity_scale)
		#raw_input('stop1')
		#print('fraction',fraction)
		#print('attempts',attempts)
		#move_group.execute(new_plan, wait=True)    
		#print('plan2',plan.joint_trajectory.points[1].time_from_start)
		#print('plan_temp',plan_temp.joint_trajectory.points[1].time_from_start)
		#raw_input('stop2')
		move_group.execute(new_plan, wait=True)    

	
    		
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
										0.02,        # eef_step
										0.02)         # jump_threshold
		move_group.execute(plan, wait=True)    

	def set_joint_velocity_scale(self, velocity_scale):
		self.velocity_scale=velocity_scale
		move_group = self.move_group
		move_group.set_max_velocity_scaling_factor(velocity_scale)

	def set_joint_acceleration_scale(self,acceleration_scale):
		self.acceleration_scale=acceleration_scale
		move_group = self.move_group
		move_group.set_max_acceleration_scaling_factor(acceleration_scale)

	def set_line_velocity_scale(self, line_velocity_scale):
		self.line_velocity_scale=line_velocity_scale

	def circle_plan_cartesian_path(self):
		move_group = self.move_group

		## BEGIN_SUB_TUTORIAL plan_cartesian_path
		##
		## Cartesian Paths
		## ^^^^^^^^^^^^^^^
		## You can plan a Cartesian path directly by specifying a list of waypoints
		## for the end-effector to go through. If executing  interactively in a
		##

		print('move to desire position')

		waypoints = []
		wpose = move_group.get_current_pose().pose
		print('current_wpose',wpose)
		theta=0
		attempts=0
		while theta<6.28:
					r=0.1
					wpose.position.x= -0.3+r*math.sin(theta)
					wpose.position.y = 0.4+r*math.cos(theta)
					wpose.position.z = 0.23
					waypoints.append(copy.deepcopy(wpose))
					theta=theta+0.3
		while attempts<100:
			(plan, fraction) = move_group.compute_cartesian_path(
										waypoints,   # waypoints to follow
										0.01,        # eef_step
										0.0,          # jump_threshold, 0 stand for can't jump
										True)           # avoid_collisions
			if fraction==1:
				break
			attempts=attempts+1
		plan_temp=copy.deepcopy(plan)
		new_plan= self.scale_trajectory_speed(plan_temp, self.line_velocity_scale)
		points=len(plan.joint_trajectory.points)
		print('points_number',points)
		#raw_input('stop')
		move_group.execute(new_plan, wait=True)    
		return plan, fraction
	
	def rotate_last_axis(self,angle):
		## Planning to a Joint Goal
		move_group = self.move_group
		joint_goal = move_group.get_current_joint_values()
		joint_goal[6] = angle
		move_group.go(joint_goal, wait=True)
		# Calling ``stop()`` ensures that there is no residual movement
		move_group.stop()
		return 0

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

def main():
	
	print("Press Ctrl-D to exit at any time")
	print("============ Press `Enter` to begin ...")
	input()
	motion= Motion()
	while not rospy.is_shutdown():
		try:
			motion.set_joint_velocity_scale(0.2)
			motion.set_joint_acceleration_scale(0.3)
			motion.set_line_velocity_scale(0.3)
			
			print("============ start0  Press `Enter` to execute a joint movement  ...")
			# input()
			output_value=motion.go_to_command_arm_joint(1.57,0,0,0,0,0,0)

			print("============ start1  Press `Enter` to execute a joint movement  ...")
			# input()
			output_value=motion.go_to_command_arm_joint(1.57, 0.74, -0.09, 0.65, 0.06, 1.74, 0.54)


			print("============Press `Enter` to execute a line movement  ...")
			# input()
			motion.go_to_line_movement(-0.2, 0.5, 0.25, -1, 0, 0, 0)
			
			print("============ plot circle  ...")
			# input()
			motion.circle_plan_cartesian_path()
			
			#print ("============  Press `Enter` to execute a line movement  (-0.3, 0.5, 0.23, -1, 0, 0, 0) ...")
			#input()
			#motion.go_to_line_movement(-0.3, 0.5, 0.23, -1, 0, 0, 0)


			print("============Press `Enter` to execute a line movement  ...")
			# input()
			motion.go_to_line_movement(-0.2, 0.5, 0.25, -1, 0, 0, 0)

			print("============  Press `Enter` to execute a line movement  ...")
			# input()
			motion.go_to_line_movement(-0.2, 0.4, 0.25, -1, 0, 0, 0)

			print("============ Press `Enter` to execute a line movement  ...")
			# input()
			motion.go_to_line_movement(-0.3, 0.4, 0.25, -1, 0, 0, 0)

			print("============ Press `Enter` to execute a line movement  ...")
			# input()
			motion.go_to_line_movement(-0.3, 0.5, 0.25, -1, 0, 0, 0)

			print("============ Press `Enter` to execute a line movement  ...")
			# input()
			motion.rotate_last_axis(-0.73)

			print("============ Press `Enter` to execute a line movement  ...")
			# input()
			motion.rotate_last_axis(0.73)

			print("============  Press `Enter` to execute a joint movement  ...")
			# input()
			motion.go_to_command_arm_joint(-0.03, 0.73, 0, 0.65, 0, 0.04, -0.03)
			
			print("============ Press `Enter` to execute a hand movement ...")
			# input()
			motion.hand_group.set_max_velocity_scaling_factor(0.6)
			motion.go_to_hand_position(0.09)

			print("============ end Press `Enter` to execute a hand movement ...")
			# input()
			motion.go_to_hand_position(0.01)



			print("============ start  Press `Enter` to execute a joint movement  ...")
			# input()
			output_value=motion.go_to_command_arm_joint(-1.57,0,0,0,0,0,0)
			#print('output_value', output_value)
			print("============ start  Press `Enter` to execute a joint movement  ...")
			# input()
			output_value=motion.go_to_command_arm_joint(0,0,0,0,0,0,0)

			input('stop process')

			'''  TEST
			#print "============setep1 Press `Enter` to set velocity 0.01 ..."
			#raw_input()
			motion.set_joint_velocity_scale(0.1)
			#motion.set_joint_acceleration_scale(0.1)
			# motion.set_line_velocity_scale(0.1)
			input('stop0 current state, press `Enter` to move!')
			print(motion.hand_group.get_current_joint_values())

			print("============ start1  Press `Enter` to execute a joint movement  ...")
			input()
			motion.go_to_command_arm_joint(0, 0, 0, 0, 0, 0, 0.54)

			print("============ start2  Press `Enter` to execute a joint movement  ...")
			input()
			motion.go_to_command_arm_joint(0, 0, 0, 0, 0, 0, 0)
			
			print("============ start1  Press `Enter` to execute a joint movement  ...")
			input()
			motion.go_to_command_arm_joint(1.57, 0.74, -0.09, 0.65, 0.06, 1.74, 0.54)

			print("============ start2  Press `Enter` to execute a joint movement  ...")
			input()
			motion.go_to_command_arm_joint(0, 0.74, -0.09, 0.65, 0.06, 1.74, 0.54)
			
			print ("============Step 3 set desire gripper position")
			input("if continue press enter")
			motion.hand_group.set_max_velocity_scaling_factor(0.6)

			motion.go_to_hand_position(0.04)

			print ("============Step 4 set desire gripper position")
			input("if continue press enter")
			motion.go_to_hand_position(0.1)
			'''
		except rospy.ROSInterruptException:
			return

if __name__ == '__main__':
	main()


#==================

'''
print("============ start1  Press `Enter` to execute a joint movement  ...")
			#raw_input()
			output_value=motion.go_to_command_arm_joint(1.57, 0.74, -0.09, 0.65, 0.06, 1.74, 0.54)

			print ("============Step 3 set desire gripper position")
			input("if continue press enter")
			motion.go_to_hand_position(0.04)

			print ("============Step 3 set desire gripper position")
			input("if continue press enter")
			motion.go_to_hand_position(0.08)


			print("============Press `Enter` to execute a line movement  ...")
			#raw_input()
			motion.go_to_line_movement(-0.2, 0.5, 0.23, -1, 0, 0, 0)
			
			print("============ plot circle  ...")
			#raw_input()
			motion.circle_plan_cartesian_path()
			
			#print ("============  Press `Enter` to execute a line movement  (-0.3, 0.5, 0.23, -1, 0, 0, 0) ...")
			#raw_input()
			#motion.go_to_line_movement(-0.3, 0.5, 0.23, -1, 0, 0, 0)


			print("============Press `Enter` to execute a line movement  ...")
			#raw_input()
			motion.go_to_line_movement(-0.2, 0.5, 0.23, -1, 0, 0, 0)

			print("============  Press `Enter` to execute a line movement  ...")
			#raw_input()
			motion.go_to_line_movement(-0.2, 0.4, 0.23, -1, 0, 0, 0)

			print("============ Press `Enter` to execute a line movement  ...")
			#raw_input()
			motion.go_to_line_movement(-0.3, 0.4, 0.23, -1, 0, 0, 0)

			print("============ Press `Enter` to execute a line movement  ...")
			#raw_input()
			motion.go_to_line_movement(-0.3, 0.5, 0.23, -1, 0, 0, 0)

			motion.rotate_last_axis(-0.73)
			motion.rotate_last_axis(0.73)

			print("============  Press `Enter` to execute a joint movement  ...")
			#raw_input()
			motion.go_to_command_arm_joint(-0.03, 0.73, 0, 0.65, 0, 0.04, -0.03)

			print("============ Press `Enter` to execute a hand movement ...")
			#raw_input()
			motion.go_to_command_hand_joint(0.03)

			print("============ end Press `Enter` to execute a hand movement ...")
			#raw_input()
			motion.go_to_command_hand_joint(0.0)



			print("============ start  Press `Enter` to execute a joint movement  ...")
			#raw_input()
			output_value=motion.go_to_command_arm_joint(0,0,0,0,0,0,0)
			#print('output_value', output_value)

			input('stop process')

'''



'''
print "============ Press `Enter` to execute a movement using a hand joint state goal ..."
raw_input()
motion.go_to_command_hand_joint(0.03,0)
'''

'''
print "============ Press `Enter` to execute a movement using a arm joint state goal ..."
raw_input()
motion.go_to_command_arm_joint(0,pi/4,0,0,0,0,0)

print "============ Press `Enter` to execute a movement using a arm joint state goal ..."
raw_input()
motion.go_to_command_arm_joint(pi/4,pi/4,0,pi/4,0,0,0)

print "============ Press `Enter` to execute a movement using a arm joint state goal ..."
raw_input()
motion.go_to_command_arm_joint(-pi/4,-pi/4,0,-pi/4,0,0,0)

print "============ Press `Enter` to execute a pose goal ..."
raw_input()
motion.go_to_pose_goal(0.2, 0.3, 0.6, 1, 0, 0, 0)

'''
'''
print "============ Press `Enter` to execute a pose goal ..."
raw_input()
motion.go_to_pose_goal(0.4, 0.3, 0.4, 0,1,0,0)

print "============ Press `Enter` to execute a line command ..."
raw_input()
motion.go_to_pose_goal(0.4, 0.3, 0.2, 0,1,0,0)

print "============ Press `Enter` to execute a line command ..."
raw_input()
motion.go_to_line_movement(0.5, 0.3, 0.2, 0,1,0,0)

print "============ Press `Enter` to execute a line command ..."
raw_input()
motion.go_to_line_movement(0.5, 0.4, 0.2, 0,1,0,0)


print "============ Press `Enter` to execute gripper movement to 0.03 ..."
raw_input()
motion.go_to_command_hand_joint(0.03)

print "============ Press `Enter` to execute gripper movement , desire  gripper position 0.0 ..."
raw_input()
motion.go_to_command_hand_joint(0.0)

print "============ Press `Enter` to execute a pose command ..."
raw_input()
motion.go_to_pose_goal(0.4, 0.3, 0.3, 0,1,0,0)

print "============ Press `Enter` to execute a movement using a arm joint state goal ..."
raw_input()
motion.go_to_command_arm_joint(pi/3,pi/3,pi/3,0,0,0,0)

print "============ Press `Enter` to execute a movement using a arm joint state goal ..."
raw_input()
motion.go_to_command_arm_joint(-pi/3,-pi/3,-pi/3,0,0,0,0)


print "============ Press `Enter` to execute a movement using a hand joint state goal ..."
raw_input()
motion.go_to_command_hand_joint(0.03)

print "============ Press `Enter` to execute a movement using a hand joint state goal ..."
raw_input()
motion.go_to_command_hand_joint(0)

'''

'''
demo 1
print "============setep1 Press `Enter` to set velocity 0.01 ..."
raw_input()
motion.set_joint_velocity_scale(0.3)

print "============ setep2 Press `Enter` to execute a joint movement  ..."
raw_input()
motion.go_to_command_arm_joint(-0.24, 0.66, -0.15, 0.57, 0.21, 1.87, -0.35)

print "============ setep3  Press `Enter` to execute a line movement  ..."
raw_input()
motion.go_to_line_movement(0.2, 0.4, 0.291, 1, 0, 0, 0)

print "============ setep4  Press `Enter` to execute a line movement  ..."
raw_input()
motion.go_to_line_movement(-0.2, 0.4, 0.291, 1, 0, 0, 0)

print "============ setep5 Press `Enter` to execute a line movement  ..."
raw_input()
motion.go_to_line_movement(-0.2, 0.4, 0.5, 1, 0, 0, 0)

print "============ setep5 Press `Enter` to execute a line movement  ..."
raw_input()
motion.go_to_line_movement(-0.2, 0.4, 0.2, 1, 0, 0, 0)

print "============ setep6 Press `Enter` to execute a joint movement  ..."
raw_input()
motion.go_to_command_arm_joint(-0.24, -0.35, -0.15, 1.05, 0.21, 0.92, -0.09)

print "============ setep7 Press `Enter` to execute a hand movement ..."
raw_input()
motion.go_to_command_hand_joint(0.03)

print "============ setep8 Press `Enter` to execute a hand movement ..."
raw_input()
motion.go_to_command_hand_joint(0.0)
'''
