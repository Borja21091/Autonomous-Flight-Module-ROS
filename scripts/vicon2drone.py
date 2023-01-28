#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg	import TFMessage

class Echo(object):

	def __init__(self):
		# Params
		self.loop_rate = rospy.Rate(10)
		# This is where you store all the data you receive
		self.value = PoseStamped()
		# Publisher
		self.pub1 = rospy.Publisher('mavros/vision_pose/pose',PoseStamped,queue_size=10)
		self.pub2 = rospy.Publisher('mavros/fake_gps/mocap/pose',PoseStamped,queue_size=10)
		# Subscriber
		rospy.Subscriber('/vicon/SUI_Endurance/SUI_Endurance',TransformStamped,self.callback)

	def callback(self,data): # data type of TransformStamped
		# Each subscriber gets 1 callback, and the callback either
		# stores information and/or computes something and/or publishes
		# It does NOT return anything
		self.value.header = data.header
		self.value.pose.position = data.transform.translation
		self.value.pose.orientation = data.transform.rotation

	def start(self):
		while not rospy.is_shutdown():
			self.pub1.publish(self.value)
			self.pub2.publish(self.value)
			self.loop_rate.sleep()

if __name__ == '__main__':
	try:
		# Init ROS Node
		rospy.init_node('vicon_listener', anonymous = True)
		rospy.loginfo('Node Started')
		echo = Echo()
		echo.start()
	except rospy.ROSInterruptException:
		pass