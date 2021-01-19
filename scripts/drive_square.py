#!/usr/bin/env python3


import rospy

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

import time
import math


speed = 0.5             ## the robot's top speed
forward_time = 5.0      ## how long the robot maintains top speed


class DriveInSquare(object):
	""" This node repeatedly drives the robot in a square """
	def __init__(self):
		## initializes the node and sets up for publishing to cmd_vel 
		## (which will change robot's angular and linear velocity)
		rospy.init_node("drive_square")
		self.twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
		r = rospy.Rate(2)
		lin = Vector3()
		ang = Vector3()
		self.twist = Twist(linear=lin, angular = ang)  ## initially stationary
	def ramp_up(self, end):
		## accelerates robot from stationary to speed 'end', pausing every 0.5s. 
		s = 0.0
		while s < end:
			s = round(s+0.1, 1)
			#print("speed:", s)
			self.twist.linear.x = s
			self.twist_pub.publish(self.twist)
			time.sleep(0.5)
	def ramp_down(self, start):
		## deccelerates robot from speed 'start' to stationary, pausing every 1s.
		s = start
		while s > 0.0:
			s = round(s-0.1, 1)
			#print("speed:", s)
			self.twist.linear.x = s
			self.twist_pub.publish(self.twist)
			time.sleep(1.0)

	def run(self):
		while not rospy.is_shutdown():
			## robot accelerates to speed in 0.1 m/s increments
			self.ramp_up(speed)
			#robot maintains speed for forward_time s
			time.sleep(forward_time)
			## robot decelerates from speed to 0 in 0.1 m/s increments
			self.ramp_down(speed)
			## robot stops for 1s before completing twist (to avoid drift)
			time.sleep(1.0)
			## robot turns 90 deg = pi/2 radians
			self.twist.angular.z = 0.5
			self.twist_pub.publish(self.twist)
			time.sleep(math.pi)
			## robot stops turning, again pausing to avoid drift
			self.twist.angular.z = 0.0
			self.twist_pub.publish(self.twist)
			time.sleep(1.0)


if __name__ == "__main__":
	node = DriveInSquare()
	node.run()
