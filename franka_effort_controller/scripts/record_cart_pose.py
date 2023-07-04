#!/usr/bin/env python3

import sys
import copy
import rospy
from math import pi
import math
from time import sleep
# import tf2_ros
import tf
import csv
import tf2_ros as tf2
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
def talker():
	rospy.init_node('tf_publisher', anonymous=True)
	pub = rospy.Publisher('tf_publisher', Float64MultiArray, queue_size=10)
	
	listener1 = tf.TransformListener()
	rate = rospy.Rate(20) # 10hz
	while not rospy.is_shutdown():
		listener1.waitForTransform('/world','/probe_ee',rospy.Time(), rospy.Duration(4.0))
		(trans1,rot1) = listener1.lookupTransform('/world', '/probe_ee',rospy.Time(0))
		roteuler1=tf.transformations.euler_from_quaternion(rot1)
		postion_array = [trans1[0],trans1[1],trans1[2],roteuler1[0],roteuler1[1],roteuler1[2]]
		position = Float64MultiArray(data=postion_array)
		pub.publish(position)
		rate.sleep()
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
