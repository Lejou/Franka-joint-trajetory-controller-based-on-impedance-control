#!/usr/bin/env python3

from __future__ import print_function
from asyncio import current_task

import copy
import rospy
import PandaMassage
from time import sleep
import numpy

def main():

	pneumatic = PandaMassage.ArduinoPneumaticActuator()

	home_point = [0.5286889490021094, -0.1389050686968094, 0.3252420799100697, 0.9998611758779029, 0.012269542854380361, -0.011253558895930662, 0.0007079367156619242]
	pneumatic.panda.set_start_point(home_point)
 
	wpose = pneumatic.panda.read_current_pose()
  
	target_waypoints_record=numpy.array([
		[0.6205915090341102, 0.21203104319476945, 0.14900043465696017, 0.9971723217010644, -0.05220700919994277, -0.014613190639473932, 0.052044730041160975],
		[0.5821171459962153, -0.1126345813110438, 0.1640927604287642, 0.9928907532459996, -0.050147812937292625, -0.03276465652892888, 0.10285668810078356],
		[0.5679848509848252, -0.21463796006978655, 0.17785687560129165, 0.9953244917645627, -0.03257535013705218, -0.01821575125269147, 0.08908293928678657]
 	])
	# print("target_waypoints_record ", target_waypoints_record)

	(row,col) = target_waypoints_record.shape
	wpose = pneumatic.panda.move_group.get_current_pose().pose
	target_waypoints = []
	interval_time_ =4

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

 
	try:
		print("Start the massage!")
		input(
			"============ Press `Enter` to move to the start point ..."
		)
		pneumatic.panda.set_joint_velocity_scale(0.1)
		pneumatic.panda.set_joint_acceleration_scale(0.1)
		pneumatic.panda.set_line_velocity_scale(0.1)
		pneumatic.panda.move_to_start()
		print("Record data!")
		pneumatic.turn_on_recording()
		# pneumatic.panda.set_line_velocity_scale(0.4)
		# cur_pose = pneumatic.panda.move_group.get_current_pose().pose
		# temp_start_pose = pneumatic.find_neighbor_pose(-0.03, cur_pose)

		# waypoints_ = []
		# waypoints_.append(copy.deepcopy(temp_start_pose))
		# trajectory_state = pneumatic.panda.go_to_waypoints(waypoints_,True)

		# pneumatic.panda.set_line_velocity_scale(0.25)
		
		for row_i in range(row):
			input(
				"============ massage pose "+ str(row_i)
			)
			# if(row_i == 0):
			# 	pneumatic.ready_waypoints_.append(copy.deepcopy(ready_grasp_pose1))
			# 	pneumatic.ready_grasp_pose = copy.deepcopy(target_waypoints[row_i])
			pneumatic.excute_cycle_massage(target_waypoints[row_i], interval_time_)

		# row_i = 3
		# pneumatic.excute_massage(target_waypoints[row_i], interval_time_)		
		# row_i = 4
		# pneumatic.excute_massage(target_waypoints[row_i], interval_time_)

		input(
			"============ Press `Enter` to back to the start point ..."
		)
		# # pneumatic.panda.set_joint_velocity_scale(0.2)
		# # pneumatic.panda.set_joint_acceleration_scale(0.2)
		pneumatic.panda.set_line_velocity_scale(0.1)
		pneumatic.panda.move_to_start()
  
	except rospy.ROSInterruptException:
		return
	except KeyboardInterrupt:
		return



if __name__ == "__main__":
	main()

