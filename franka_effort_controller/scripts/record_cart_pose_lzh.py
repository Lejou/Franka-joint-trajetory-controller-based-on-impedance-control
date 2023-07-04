#!/usr/bin/env python3

# import sys
# import copy
import rospy
# import moveit_commander
# import moveit_msgs.msg
# import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
# from moveit_commander.conversions import pose_to_list
import math
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from time import sleep
# import tf2_ros
import tf
import csv

if __name__ == '__main__':
	rospy.init_node('tf_listener', anonymous=True)
	listener1 = tf.TransformListener()
	rate=rospy.Rate(100.0)
	# pub = rospy.Publisher('ee_xyz', geometry_msgs.Vector3, queue_size=10)
	# pose = geometry_msgs.msg.Pose()
	rows = []
	i=0

	try:
		while not rospy.is_shutdown():
			input(
			"============ Press `Enter` to record the pose ..."
			)
			try:
				(trans1,rot1) = listener1.lookupTransform('/world', '/probe_ee',rospy.Time(0)) # trans1:x,y,z; rot1:x,y,z,w
				roteuler1=tf.transformations.euler_from_quaternion(rot1,axes='rxyz')

				if i ==0:
					roteuler_base=tf.transformations.euler_from_quaternion(rot1,axes='rxyz')
					rot_p=tf.transformations.euler_from_quaternion(rot1,axes='rxyz')

				# rot_p = tuple(map(lambda i,j: i - j, roteuler1, roteuler_base))
				# print("trans1",trans1)
				now = rospy.get_rostime()
				sec_now = now.to_sec() 
				# print("sec_now",sec_now)
				listOfStr = ['[',trans1[0],trans1[1],trans1[2],rot1[0],rot1[1],rot1[2],rot1[3],']']
				# listOfStr = [sec_now,trans1[0],trans1[1],trans1[2],rot_p[0],rot_p[1],rot_p[2],rot1[0],rot1[1],rot1[2],rot1[3]]
				rows.append(listOfStr)
				# rot_p = tuple(map(lambda i,j: i - j, roteuler1, roteuler_base))
				# print("rot_p:",rot_p)
				# for i in range(1, 4):
				# 	writer.writerow([trans1[i-1]])
				# 	print("trans1[i-1] ",trans1[i-1])
				# for i in range(4, 7):
				# 	writer.writerow([roteuler1[i-4]])
				
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				continue
			rate.sleep()
			i +=1
	except KeyboardInterrupt:
		pass

	with open('/home/robot/a_panda_ws/src/franka_controllers/record_data/20221216/cart_pose_read'+str(now.secs)+'.csv', 'w') as csvfile:
		writer = csv.writer(csvfile, delimiter=',')
		writer.writerows(rows)
	csvfile.close()

