#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
from math import pi
from std_msgs.msg import Int16
import math
from time import sleep
import numpy as np

class PressureControl(object):

	def __init__(self):
		# super(armJoint, self).__init__()

		rospy.init_node('pressure_control', anonymous=True)
		self.pwm_pub=rospy.Publisher('/pressure_pwm',Int16,queue_size=10)
		self.pwm_vel_sub=rospy.Subscriber('/pressure_value',Int16,self.pwm_vel_callback,buff_size=10)
		self.pressure_sub=rospy.Subscriber('/pressure_value',Int16,self.pressure_value_callback,buff_size=10)

	def pressure_value_callback(self,data):
		self.pressure_value = data.data
		print("pressure_value: ",self.pressure_value)
	
	def pwm_vel_callback(self,data):
		self.pwm_vel = data.data
		print("pwm_vel: ",self.pwm_vel)
	

def main():
	pc_=PressureControl()
	rospy.spin()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
