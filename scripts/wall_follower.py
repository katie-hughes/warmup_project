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
dist = 0.5
buffer = 0.2
delta = 5  ##allowed angle offset
speed = 0.2

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
        minimum_angle = np.argmin(np.array(data.ranges))
        minimum_distance = data.ranges[minimum_angle]
        front_distance = data.ranges[0]
        print("the minimum distance is", minimum_distance)
        print("the minimum angle is", minimum_angle)
        if dist-buffer <= minimum_distance <= dist+buffer:
            print("The robot is the correct distance from the wall")
            if 270-delta <= minimum_angle <= 270+delta:
                print("The robot is facing the right way")
                error4 = 270 - minimum_angle
                kp4 = 0.01
                self.twist.linear.x = speed
                self.twist.angular.z = error4*kp4
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
                print("ang velocity: ", error1*kp1)
                self.twist.linear.x = 0
                self.twist.angular.z = kp1*error1
                self.twist_pub.publish(self.twist)
        elif minimum_distance > dist+buffer:
            print("The robot is too far away")
            if minimum_angle <= delta or minimum_angle >= (360-delta):
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
        else:
            print("The robot is too close")
            ##robot needs to turn away from the wall
            if minimum_angle < 300:
                print("Robot needs to trun away from the wall ")
                error5 = 300 - minimum_angle
                kp5 = 0.01
                self.twist.linear.x = 0
                self.twist.angular.z = error5*kp5
                self.twist_pub.publish(self.twist)
            else:
                self.twist.linear.x = speed
                self.twist.angular.z = 0
                self.twist_pub.publish(self.twist)


    def run(self):
        rospy.spin()


def main():
    node = FollowWall()
    node.run()

if __name__ == "__main__":
    main()
