#!/usr/bin/env python3
""" Description """
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import LaserScan
import numpy as np
import time
import math


dist = 0.4

delta = 15

class FollowPerson(object):
    """ description """
    def __init__(self):
        rospy.init_node("follow_wall")
        rospy.Subscriber("/scan", LaserScan, self.process_scan)
        self.twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
        r = rospy.Rate(2)
        lin = Vector3()
        ang = Vector3()
        self.twist = Twist(linear=lin, angular = ang)
    def process_scan(self, data):
        ## len of data.ranges is 360
        print()
        minimum_angle = np.argmin(np.array(data.ranges))
        minimum_distance = data.ranges[minimum_angle]
        front_distance = data.ranges[0]
        print("the minimum distance is", minimum_distance)
        print("the minimum angle is", minimum_angle)
        if minimum_distance == float('inf'):
            print("person not found")
            self.twist.linear.x = 0
            self.twist.angular.z = 0
            self.twist_pub.publish(self.twist)
        elif minimum_distance > dist:
            kp_speed = 0.5
            kp_angle = 0.02
            error_speed = minimum_distance - dist
            error_angle = 0
            if minimum_angle <= 180:
                print('The person is on the left')
                error_angle = minimum_angle
            else:
                print('The person is on the right')
                error_angle =minimum_angle - 360
            print("Error angle is", error_angle)
            self.twist.linear.x = error_speed*kp_speed
            if delta <= minimum_angle <= (360-delta):
                self.twist.linear.x = 0
            self.twist.angular.z = error_angle*kp_angle
            self.twist_pub.publish(self.twist)
        else:
            self.twist.linear.x = 0
            self.twist.angular.z = 0
            self.twist_pub.publish(self.twist)

    def run(self):
        rospy.spin()


def main():
    node = FollowPerson()
    node.run()

if __name__ == "__main__":
    main()
