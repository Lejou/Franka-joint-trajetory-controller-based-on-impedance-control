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
from std_msgs.msg import Float32MultiArray,Float32,Float64,Int16,Bool
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import Quaternion, Pose, Point
from transforms3d.quaternions import quat2mat, mat2quat
from franka_msgs.msg import FrankaState
from time import sleep
# import tf2_ros
import tf
import csv
import numpy
import copy

class DataRecord(object):
	def __init__(self):
		self.strain_gauge_val_ = []
		strain_gauge_sub = rospy.Subscriber("strain_gauge_values",Float32MultiArray,self.strain_gauge_callback,queue_size=10) # Get vertical force
		self.strain_gauge_pub = rospy.Publisher("strain_gauge_force", Float32MultiArray, queue_size=10)
		
		self.external_force = []
		self.desire_force = 0
		self.target_force = 0

		ext_force_sub = rospy.Subscriber("/franka_state_controller/F_ext",WrenchStamped,self.ext_force_callback,queue_size=10) # Get external force
		desire_force_sub = rospy.Subscriber("/franka_force_joint_trajectory_controller/force_control_desired_force",Float64,self.desire_force_callback,queue_size=10) # Get external force
		target_force_sub = rospy.Subscriber("/franka_force_joint_trajectory_controller/target_force",Float64,self.target_force_callback,queue_size=10) # Get external force


		self.cur_ee_pose= [] # current ee pose: xyz, qx,qy,qz,qw
		ee_mat_sub = rospy.Subscriber("/franka_state_controller/franka_states",FrankaState,self.frank_ee_callback,queue_size=10) # Get ee 

		self.csv_rows = []
		self.sg_ratio = numpy.array([3.25,3.27,2.40,3.1,3,2.37])
		self.base_sg_value = numpy.arange(6)
		self.sealing_force = numpy.arange(6)

		self.main_pressure_ =[]
		self.main_pressure_ref_ =0
		self.lip_pressure_ = []
		self.lip_pressure_sub_ = -100

		self.data_record_state = False
		lip_pressure_value_sub = rospy.Subscriber("lip_pressure_val",Float32,self.lip_pressure_value_callback,queue_size=10) # Get lip pressure
		lip_pressure_ref_sub = rospy.Subscriber("lip_pressure_sub",Float32,self.lip_pressure_ref_callback,queue_size=10) # Get lip pressure
		main_pressure_value_sub = rospy.Subscriber("main_pressure_val",Float32,self.main_pressure_value_callback,queue_size=10)
		main_pressure_ref_sub = rospy.Subscriber("main_valve_threshold",Int16,self.main_pressure_ref_callback,queue_size=10)
		data_record_sub = rospy.Subscriber("data_record_topic",Bool,self.data_record_callback,queue_size=10)

	def data_record_callback(self,data):
		# self.strain_gauge_val_ = [data.data[0],data.data[1],data.data[2],data.data[3],data.data[4],data.data[5]]
		self.data_record_state =data.data
		print("Record data received:",data.data)


	def desire_force_callback(self,data):
		self.desire_force =data.data
	def target_force_callback(self,data):
		self.target_force =data.data


	def strain_gauge_callback(self,data):
		# self.strain_gauge_val_ = [data.data[0],data.data[1],data.data[2],data.data[3],data.data[4],data.data[5]]
		self.strain_gauge_val_ =numpy.array(data.data)

	def ext_force_callback(self,data):
		self.external_force = [data.wrench.force.x,data.wrench.force.y,data.wrench.force.z,data.wrench.torque.x,data.wrench.torque.y,data.wrench.torque.z]

	def frank_ee_callback(self,franka_state):
		cur_ee_float_array = franka_state.O_T_EE
		cur_ee_mat=numpy.eye(4)
		cur_ee_mat = numpy.array([ [cur_ee_float_array[0], cur_ee_float_array[4], cur_ee_float_array[8],cur_ee_float_array[12]], 
				 						[cur_ee_float_array[1], cur_ee_float_array[5], cur_ee_float_array[9],cur_ee_float_array[13]], 
										[cur_ee_float_array[2], cur_ee_float_array[6], cur_ee_float_array[10],cur_ee_float_array[14]], 
										[cur_ee_float_array[3], cur_ee_float_array[7], cur_ee_float_array[11],cur_ee_float_array[15]]])
		quat = mat2quat(cur_ee_mat[:3, :3])
		self.cur_ee_pose = [cur_ee_mat[0, 3],cur_ee_mat[1, 3],cur_ee_mat[2, 3],quat[1], quat[2], quat[3],quat[0]]

	def get_base_sg_value(self):
		self.base_sg_value = copy.deepcopy(self.strain_gauge_val_)
		# print("self.base_sg_value ",self.base_sg_value)

	def get_sealing_force(self):
		# print("strain_gauge_val_: ",self.strain_gauge_val_)
		# print("base_sg_value: ",self.base_sg_value)
		# print("sg_ratio: ",self.sg_ratio)
		delta_sg_value = (self.strain_gauge_val_ - self.base_sg_value)
		self.sealing_force =self.sg_ratio * delta_sg_value

		strain_gauge_force_msg = Float32MultiArray()
		strain_gauge_force_msg.data = self.sealing_force
		self.strain_gauge_pub.publish(strain_gauge_force_msg)


	def main_pressure_value_callback(self,data):
		# print(-data.data * 7.5)
		self.main_pressure_ = int(-data.data * 7.5) # unit: mm/Hg
		# self.main_pressure_ = math.ceil(-data.data * 7.5)

	def main_pressure_ref_callback(self,data):
		# print(-data.data * 7.5)
		self.main_pressure_ref_ = int(-data.data) # unit: mm/Hg
		# self.main_pressure_ = math.ceil(-data.data * 7.5)

	def lip_pressure_value_callback(self,data):
		# print(data.data)
		self.lip_pressure_ = data.data

	def lip_pressure_ref_callback(self,data):
		# print(data.data)
		self.lip_pressure_sub_ = data.data

	def record_data(self):
		now = rospy.get_rostime()
		sec = now.secs%1000
		nsec =now.nsecs//1000000
		time_cur = sec+nsec/1000
		if self.cur_ee_pose != []:
			listOfStr = [str(time_cur),
						'cur_ee_pose',self.cur_ee_pose[0],self.cur_ee_pose[1],self.cur_ee_pose[2],self.cur_ee_pose[3],self.cur_ee_pose[4],self.cur_ee_pose[5],self.cur_ee_pose[6],
						'external_force',self.external_force[0],self.external_force[1],self.external_force[2],self.external_force[3],self.external_force[4],self.external_force[5],
						'strain_gauge_val',self.strain_gauge_val_[0],self.strain_gauge_val_[1],self.strain_gauge_val_[2],self.strain_gauge_val_[3],self.strain_gauge_val_[4],self.strain_gauge_val_[5],
						'sealing_force',self.sealing_force[0],self.sealing_force[1],self.sealing_force[2],self.sealing_force[3],self.sealing_force[4],self.sealing_force[5],
						'main_pressure',self.main_pressure_,
						'main_pressure_ref',self.main_pressure_ref_,
						'lip_pressure',self.lip_pressure_,
						'lip_pressure_sub_',self.lip_pressure_sub_,
						'desire_force',self.desire_force,
						'target_force',self.target_force
						]
			self.csv_rows.append(listOfStr)

	def record_data_without_panda(self):
		now = rospy.get_rostime()
		sec = now.secs%1000
		nsec =now.nsecs//1000000
		time_cur = sec+nsec/1000
		listOfStr = [str(time_cur),
					'strain_gauge_val',self.strain_gauge_val_[0],self.strain_gauge_val_[1],self.strain_gauge_val_[2],self.strain_gauge_val_[3],self.strain_gauge_val_[4],self.strain_gauge_val_[5],
					'sealing_force',self.sealing_force[0],self.sealing_force[1],self.sealing_force[2],self.sealing_force[3],self.sealing_force[4],self.sealing_force[5],
					'main_pressure',self.main_pressure_,
					'main_pressure_ref',self.main_pressure_ref_,
					'lip_pressure',self.lip_pressure_,
					'lip_pressure_sub_',self.lip_pressure_sub_,
					'desire_force',self.desire_force,
					'target_force',self.target_force
					]
		self.csv_rows.append(listOfStr)


	def save_data(self,filestr):

		with open(filestr, 'w') as csvfile:
			writer = csv.writer(csvfile, delimiter=',')
			writer.writerows(self.csv_rows)
		csvfile.close()

	def transform_matrix_to_ros_pose(self,mat):
		"""
		Convert a transform matrix to a ROS pose.
		"""
		quat = mat2quat(mat[:3, :3])
		msg = Pose()
		msg.position = Point(x=mat[0, 3], y=mat[1, 3], z=mat[2, 3])
		msg.orientation = Quaternion(w=quat[0], x=quat[1], y=quat[2], z=quat[3])
		return msg

if __name__ == '__main__':
	rospy.init_node('data_record_node', anonymous=True)
	dr = DataRecord()

	rate=rospy.Rate(10.0)
	sg_init_state = False
	try:
		while not rospy.is_shutdown():
			rate.sleep()
			if(dr.data_record_state):
				if(not sg_init_state):
					sg_init_state=True
					dr.get_base_sg_value()
				dr.get_sealing_force()
				dr.record_data()
				# dr.record_data_without_panda()
	except KeyboardInterrupt:
		pass
	now = rospy.get_rostime()
	# filestr = '/home/robot/a_panda_ws/src/franka_controllers/record_data/20230404/baseExp_main_pressure_control_'+str(now.secs)+'.csv'
	# filestr = '/home/robot/a_panda_ws/src/franka_controllers/record_data/20230615/baseExp_adaptive_control_r20_'+str(now.secs%1000000)+'.csv'
	# filestr = '/home/robot/a_panda_ws/src/franka_controllers/record_data/20230615/head_lip_size_variation_without_panda_'+str(now.secs)+'.csv'
	# filestr = '/home/robot/a_panda_ws/src/franka_controllers/record_data/20230619/SG_SinglePointTrigger_'+str(now.secs%1000000)+'.csv'
	# filestr = '/home/robot/a_panda_ws/src/franka_controllers/record_data/20230625/SG_LipRoundTrigger_'+str(now.secs%1000000)+'.csv'
	filestr = '/home/robot/a_panda_ws/src/franka_controllers/record_data/20230704/demo_phantom_exp_'+str(now.secs%1000000)+'.csv'
	dr.save_data(filestr)
	
	
	# with open('/home/robot/a_panda_ws/src/franka_controllers/record_data/20230315/baseExp_sg_record_'+str(now.secs)+'.csv', 'w') as csvfile:
	# 	writer = csv.writer(csvfile, delimiter=',')
	# 	writer.writerows(dr.csv_rows)
	# csvfile.close()
 