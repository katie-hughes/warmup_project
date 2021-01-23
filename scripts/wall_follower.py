#!/usr/bin/env python3
""" Description """
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import LaserScan
import numpy as np
import time
import math


angles = np.arange(360)
dist = 0.3
buffer = 0.15
delta = 10

class FollowWall(object):
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
        minimum = np.argmin(np.array(data.ranges))
        minimum_distance = data.ranges[minimum]
        minimum_angle = angles[minimum]
        #print("the minimum index is", minimum)
        print("the minimum distance is", minimum_distance)
        print("the minimum angle is", minimum_angle)
        """
        Now what do I want the robot to do?
        I know what direction it needs to turn to face the closest wall
        The first step is to drive that direction
        which means turn towards the appropriate angle and also drive towards
        Once the robot gets a certain distance away from the wall it should
        just drive straight (say 0.4 m or something).
        If driving along straight wall
        the goal angle should be 90 +/- delta.
        """
        if dist-buffer <= minimum_distance <= dist+buffer:
            print("The robot is the correct distance from the wall")
            if 270-delta <= minimum_angle <= 270+delta:
                print("The robot is facing the right way")
                self.twist.linear.x = 0.1
                self.twist.angular.z = 0
                self.twist_pub.publish(self.twist)
            else:
                print("The robot needs to turn")
                error1 = 0
                if 90<=minimum_angle<=270:      ##turn right
                    print("Turn right")
                    error1 = minimum_angle - 270
                else:
                    print("Turn left")
                    if minimum_angle > 0:
                        error1 = minimum_angle - 270
                    else:
                        error1 = 90 + minimum_angle
                kp1 = 0.01
                print("ang velocity: ", error1)
                self.twist.linear.x = 0
                self.twist.angular.z = kp1*error1
                self.twist_pub.publish(self.twist)
        else:
            print("The robot is too far away from the wall")
            ##robot needs to turn to face the wall
            if minimum_angle <= 10 or minimum_angle >= 350:
                print("The robot is facing the appropriate wall")
                error2 = minimum_distance - dist
                kp2 = 1.0
                self.twist.linear.x = kp2*error2
                self.twist.angular.z = 0
                self.twist_pub.publish(self.twist)
            else:
                print("The robot needs to turn to face the appropriate wall")
                error3 = 0
                if minimum_angle <= 180:    ## closest wall on the left
                    print("closest wall is on the left")
                    error3 = minimum_angle
                else:                       ## closest wall on the right
                    error3 = minimum_angle - 360
                    print("closest wall is on the right")
                kp3 = 0.01
                print("error3 is", error3)
                print("new angular velocity is", kp3*error3)
                self.twist.linear.x = 0
                self.twist.angular.z = kp3*error3
                self.twist_pub.publish(self.twist)
                ## i think this portion generally works.

    def run(self):
        rospy.spin()


def main():
    node = FollowWall()
    node.run()

if __name__ == "__main__":
    main()
