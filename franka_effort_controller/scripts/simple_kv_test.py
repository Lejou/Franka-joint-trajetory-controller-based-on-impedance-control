#!/usr/bin/env python3

import sys
import copy
import rospy
import math
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from std_msgs.msg import Bool

from kivymd.app import MDApp
from kivymd.uix.floatlayout import MDFloatLayout
from kivymd.icon_definitions import md_icons

from kivy.lang import Builder
from kivy.config import Config
Config.set('kivy','keyboard_mode','systemanddock')
from kivy.core.window import Window
Window.size = (1000, 750)
from kivymd.uix.screen import MDScreen

class SimpleKV(MDApp):

	def __init__(self,**kwargs):
		self.main_pwm = 0
		super().__init__(**kwargs)
		self.screen = Builder.load_file('/home/lee/robot/src/th_demo/config/TestKivy.kv')
		self.main_slider_pub = rospy.Publisher("main_pressure_pwm",Int16,queue_size=10)
		self.main_valve_pub = rospy.Publisher("main_valve_switch",Int16,queue_size=10)
		self.main_interval_freq_pub = rospy.Publisher("main_interval_freq",Float32,queue_size=10)
		self.main_interval_duty_pub = rospy.Publisher("main_interval_duty_cycle",Float32,queue_size=10)
		self.main_valve_threshold_pub = rospy.Publisher("main_valve_threshold",Int16,queue_size=10)
		self.main_pressure_value_sub = rospy.Subscriber("main_pressure_val",Float32,self.pressure_value_callback,queue_size=10)

	def build(self):
		return self.screen

	def main_slider_function(self,slider_value):
		# print(int(slider_value))
		self.main_pwm = int(slider_value)
		msg = Int16()
		msg.data = int(slider_value)
		self.main_slider_pub.publish(msg)

	def main_valve_threshold_function(self,value):
		# print(int(slider_value))
		msg = Int16()
		msg.data = -math.ceil(value/7.5)
		self.main_valve_threshold_pub.publish(msg)

	def main_interval_freq_function(self,value):
		# print(int(slider_value))
		msg = Float32()
		msg.data = round(value/10,1)
		self.main_interval_freq_pub.publish(msg)

	def main_interval_duty_cycle_function(self,value):
		# print(int(slider_value))
		msg = Float32()
		msg.data = round(value/100,2)
		self.main_interval_duty_pub.publish(msg)

	def main_valve_successive(self):
		msg = Int16()
		msg.data = 1 
		self.main_valve_pub.publish(msg)

	def main_valve_interval(self):
		msg = Int16()
		msg.data = 2
		self.main_valve_pub.publish(msg)

	def main_valve_off(self):
		msg = Int16()
		msg.data = 0
		self.main_valve_pub.publish(msg)

	def pressure_value_callback(self,data):
		# print(-data.data * 7.5)
		self.root.ids.main_pressure_value.value = int(-data.data * 7.5)
		# self.root.ids.main_pressure_value.value = math.ceil(-data.data * 7.5)

if __name__ == '__main__':

	rospy.init_node('SimpleKV', anonymous=True)
	SimpleKV().run()
	
