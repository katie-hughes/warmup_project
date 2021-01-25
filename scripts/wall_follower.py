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
delta = 10  ##allowed angle offset
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
    def change_speed(self, linear_velocity, angular_velocity):
        self.twist.linear.x = linear_velocity
        self.twist.angular.z = angular_velocity
        self.twist_pub.publish(self.twist)
    def process_scan(self, data):
        print()
        minimum_angle = np.argmin(np.array(data.ranges))
        minimum_distance = data.ranges[minimum_angle]
        front_distance = data.ranges[0]
        front_right_distance = data.ranges[345]
        print("the minimum distance is", minimum_distance)
        print("the minimum angle is", minimum_angle)
        print("the front distance is", front_distance)
        if front_distance < (dist-buffer) or front_right_distance < (dist-buffer):
            print("TOO CLOSE")
            self.change_speed(0, 0.8)
        else:
            if dist-buffer <= minimum_distance <= dist+buffer:
                print("The robot is the correct distance from the wall")
                if 270-delta <= minimum_angle <= 270+delta:
                    print("The robot is facing the right way")
                    error1 = minimum_angle - 270
                    kp1 = 0.005
                    self.change_speed(speed, error1*kp1)
                else:
                    ## The robot needs to turn
                    print("The robot needs to turn")
                    error2 = 0
                    if 90<=minimum_angle<=270:
                        ## The wall is behind the robot. Turn Right (- ang mom)
                        print("")
                        error2 = minimum_angle - 270
                    else:
                        ## The wall is in front of the robot. Turn Left (+ ang mom)
                        if minimum_angle > 270:
                            print("angle", minimum_angle)
                            error2 = minimum_angle - 270
                            print("error2:", error2)
                        else:
                            error2 = 90 + minimum_angle
                    kp2 = 0.01
                    print("ang velocity: ", error2*kp2)
                    self.change_speed(0, kp2*error2)
            elif minimum_distance > dist+buffer :
                ## The robot is too far away from the wall
                print("The robot is too far away")
                kp_speed = 0.3
                kp_angle = 0.02
                error_speed = minimum_distance   # - dist
                error_angle = 0
                if minimum_angle <= 180:
                    ## The closest wall is on the left, robot needs to turn left
                    print('The closest wall is on the left')
                    error_angle = minimum_angle
                else:
                    ## The closest wall is on the right, robot needs to turn right
                    print('The closest wall is on the right')
                    error_angle =minimum_angle - 360
                print("Error angle is", error_angle)
                if delta <= minimum_angle <= (360-delta):
                    ## robot pauses until it is facing the wall
                    self.change_speed(0, error_angle*kp_angle)
                else:
                    ## robot drives straight towards the closest wall
                    self.change_speed(error_speed*kp_speed, error_angle*kp_angle)

            else:
                print("The robot is too close")
                if front_distance < (dist+buffer) or front_right_distance < (dist+buffer):
                    ## there is something immediately in front of the robot
                    print("Something in front of robot")
                    self.change_speed(0, 0.5)
                else:
                    print("Something to side")
                    # the robot's right side is very close to the wall.
                    self.change_speed(speed, 0.1)


    def run(self):
        rospy.spin()


def main():
    node = FollowWall()
    node.run()

if __name__ == "__main__":
    main()
