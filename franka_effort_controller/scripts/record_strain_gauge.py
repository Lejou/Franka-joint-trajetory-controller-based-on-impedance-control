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
from std_msgs.msg import Float32MultiArray

class ArduinoStrainGaugeRecord(object):
	def __init__(self):
		self.strain_gauge_val = []
		strain_gauge_sub = rospy.Subscriber("strain_gauge_values",Float32MultiArray,self.strain_gauge_callback,queue_size=10) # Get vertical force

	def strain_gauge_callback(self,data):
		self.strain_gauge_val = [data.data[0],data.data[1],data.data[2],data.data[3],data.data[4],data.data[5]]

def main():
	rospy.init_node('strain_data_recorder', anonymous=True)
	sg_recorder = ArduinoStrainGaugeRecord()
	rows = []
	try:
		while not rospy.is_shutdown():
			input("============ Press `Enter` to record the pose ..." )
			postion_array = sg_recorder.strain_gauge_val
			rows.append(postion_array)
	except KeyboardInterrupt:
		pass

	with open('/home/robot/a_panda_ws/src/franka_controllers/record_data/20230305/cart_pose_read'+str(now.secs)+'.csv', 'w') as csvfile:
		writer = csv.writer(csvfile, delimiter=',')
		writer.writerows(rows)
	csvfile.close()
        
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
