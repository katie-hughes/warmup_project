#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import LaserScan
import numpy as np
import time
import math

dist = 0.5              ## desired distance from wall
buffer = 0.2            ## allowed offset from dist
delta = 10              ## allowed angle offset
speed = 0.2             ## speed of the robot

class FollowWall(object):
    """ Drives robot at 'dist' away from the wall of a rectangular room.
        The robot always drives with the wall against its right side.
    """
    def __init__(self):
        rospy.init_node("follow_wall")
        ## subscribe to data from robot's sensors
        rospy.Subscriber("/scan", LaserScan, self.process_scan)
        ## set up publishing node to cmd_velocity to move/rotate robot
        self.twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
        r = rospy.Rate(2)
        lin = Vector3()
        ang = Vector3()
        self.twist = Twist(linear=lin, angular = ang)
    def change_speed(self, linear_velocity, angular_velocity):
        ## Change the linear and angular velocity of the robot
        self.twist.linear.x = linear_velocity
        self.twist.angular.z = angular_velocity
        self.twist_pub.publish(self.twist)
    def process_scan(self, data):
        ## Read in sensor data and determine how to move the robot.
        ## The angle and distance to the closest object to the robot:
        minimum_angle = np.argmin(np.array(data.ranges))
        minimum_distance = data.ranges[minimum_angle]
        ## The distance in front of the robot:
        front_distance = data.ranges[0]
        ## The distance in front + slightly to the right of the robot (345 deg):
        front_right_distance = data.ranges[345]
        if front_distance < (dist-buffer):
            ## Robot is very close to hitting the wall in front of it!
            ## Stop the robot and rotate until it is no longer in danger of crashing.
            self.change_speed(0, 0.8)
        else:
            if dist-buffer <= minimum_distance <= dist+buffer:
                ## The robot is the correct distance from the wall.
                if 270-delta <= minimum_angle <= 270+delta:
                    ## Robot correctly oriented with the wall on its right
                    error1 = minimum_angle - 270
                    kp1 = 0.005
                    self.change_speed(speed, error1*kp1)
                else:
                    ## The robot needs to turn so that the closest wall is at
                    ## an angle of 270 (on the right hand side)
                    error2 = 0
                    if 90<=minimum_angle<=270:
                        ## Wall behind the robot, turn right (error, ang mom<0)
                        error2 = minimum_angle - 270
                    else:
                        ## Wall in front of the robot, turn Left (error, ang mom>0)
                        if minimum_angle > 270:
                            error2 = minimum_angle - 270
                        else:
                            error2 = 90 + minimum_angle
                    kp2 = 0.01
                    self.change_speed(0, error2*kp2)
            elif minimum_distance > dist+buffer :
                ## The robot is too far away from the wall
                ## Usually the case when the robot begins close to room's middle
                kp_speed = 0.3
                kp_angle = 0.02
                error_speed = minimum_distance
                error_angle = 0
                if minimum_angle <= 180:
                    ## The closest wall is on the left, robot needs to turn left
                    ## (error, ang mom > 0)
                    error_angle = minimum_angle
                else:
                    ## The closest wall is on the right, robot needs to turn right
                    ## (error, ang mom < 0)
                    error_angle =minimum_angle - 360
                if delta <= minimum_angle <= (360-delta):
                    ## robot rotates in place until it is facing the wall
                    self.change_speed(0, error_angle*kp_angle)
                else:
                    ## robot is facing wall, drives straight towards it
                    self.change_speed(error_speed*kp_speed, error_angle*kp_angle)
            else:
                ## The robot is too close to the wall
                if front_distance < (dist+buffer) or front_right_distance < (dist+buffer):
                    ## there is something immediately in front of the robot
                    ## stop robot and rotate
                    self.change_speed(0, 0.5)
                else:
                    # the robot's right side is very close to the wall.
                    # move forward while rotating.
                    self.change_speed(speed, 0.1)
    def run(self):
        rospy.spin()


def main():
    node = FollowWall()
    node.run()

if __name__ == "__main__":
    main()
