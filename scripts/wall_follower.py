#!/usr/bin/env python3
""" Description """
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3


from sensor_msgs.msg import LaserScan

import numpy as np


class FollowWall(object):
    """ description """
    def __init__(self):
        rospy.init_node("follow_wall")
        rospy.Subscriber("/scan", LaserScan, self.process_scan)
        self.twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
        r = rospy.Rate(2)
        lin = Vector3()
        ang = Vector3()
        self.twist = Twist(linear=lin, angular = ang)  ## initially stationary
    def process_scan(self, data):
        print(data.ranges)
        print(len(data.ranges))
        ## len of data.ranges is 360
        angles = np.arange(360)
        minimum = np.argmin(np.array(data.ranges))
        minimum_distance = data.ranges[minimum]
        minimum_angle = angles[minimum]
        print("the minimum index is", minimum)
        print("the minimum distance is", minimum_distance)
        print("the minimum angle is", minimum_angle)
    def run(self):
        rospy.spin()


def main():
    node = FollowWall()
    node.run()

if __name__ == "__main__":
    main()
