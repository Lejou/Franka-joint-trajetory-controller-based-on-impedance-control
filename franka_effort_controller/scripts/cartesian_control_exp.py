#!/usr/bin/env python3

from __future__ import print_function
from asyncio import current_task

import copy
import rospy
import PandaMassage
from time import sleep


def main():

	pneumatic = PandaMassage.ArduinoPneumaticActuator()
	wpose = pneumatic.panda.read_current_pose()
	# print("current_pose",wpose)
	pneumatic.panda.set_joint_velocity_scale(0.2)
	pneumatic.panda.set_joint_acceleration_scale(0.1)

	try:
		print("Press Ctrl-D to exit at any time")
		input(
			"============ 1 Press `Enter` to move to the start point ..."
		)
		pneumatic.panda.move_to_start()

		while " " != input("============ 2 Press `Enter` to repeat a trajectory ..."):
			waypoints = []
			wpose = pneumatic.panda.move_group.get_current_pose().pose
			wpose.position.x = 0.45
			wpose.position.y = 0
			wpose.position.z = 0.3501
			waypoints.append(copy.deepcopy(wpose))
			
			wpose.position.y = 0.2
			waypoints.append(copy.deepcopy(wpose))

			wpose.position.y = -0.201
			waypoints.append(copy.deepcopy(wpose))
			
			wpose.position.y = 0.2
			waypoints.append(copy.deepcopy(wpose))

			wpose.position.y = -0.201
			waypoints.append(copy.deepcopy(wpose))
			
			# wpose.position.y = 0
			# wpose.position.x = 0.3501
			# waypoints.append(copy.deepcopy(wpose))

			# wpose.position.x = 0.5501
			# waypoints.append(copy.deepcopy(wpose))

			# wpose.position.x = 0.35
			# waypoints.append(copy.deepcopy(wpose))

			# wpose.position.x = 0.55
			# waypoints.append(copy.deepcopy(wpose))

			# wpose.position.x = 0.45
			# waypoints.append(copy.deepcopy(wpose))

			pneumatic.panda.go_to_waypoints(waypoints,True)

		input(
			"============ 3 Press `Enter` to move to the start point ..."
		)
		pneumatic.panda.move_to_start()
		print("============ Python tutorial demo complete!")
	except rospy.ROSInterruptException:
		return
	except KeyboardInterrupt:
		return



if __name__ == "__main__":
	main()

