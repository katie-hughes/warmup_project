#!/usr/bin/env python3
import rospy

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

import time
import math

class DriveInSquare(object):
	""" Description """
	def __init__(self):
		rospy.init_node("drive_square")
		#rospy.Subscriber("/time", float, time)
		self.twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
		r = rospy.Rate(2)
		lin = Vector3()
		ang = Vector3()
		self.twist = Twist(linear=lin, angular = ang)
		print(self.twist)
	def run(self):
		print("running")
		while not rospy.is_shutdown():
			self.twist.linear.x = 0.1
			self.twist_pub.publish(self.twist)
			print(self.twist)
			print("sleeping for 5")
			time.sleep(5.0)

			self.twist.linear.x = 0.0
			self.twist_pub.publish(self.twist)
			self.twist.angular.z = 0.5
			self.twist_pub.publish(self.twist)
			print(self.twist)
			print("sleeping for pi")
			time.sleep(math.pi)

			self.twist.angular.z = 0.0
			self.twist_pub.publish(self.twist)
			print(self.twist)
			print('successfully turned')
		#rospy.spin()

if __name__ == "__main__":
	node = DriveInSquare()
	node.run()
