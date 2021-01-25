#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import LaserScan
import numpy as np
import time
import math


dist = 0.5          ## The desired distance from the object
delta = 15          ## Allowed angle offset when determining angle to object

class FollowPerson(object):
    """ Robot locates the closest object (person) to itself
        and travels towards it, stopping at 'dist' m. away from it.
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
    def change_velocity(self, linear_velocity, angular_velocity):
        ## Change the linear and angular velocity of the robot
        self.twist.linear.x = linear_velocity
        self.twist.angular.z = angular_velocity
        self.twist_pub.publish(self.twist)
    def process_scan(self, data):
        ## Reads the sensor data to determine how to move
        ## The angle and distance to the closest object (person):
        minimum_angle = np.argmin(np.array(data.ranges))
        minimum_distance = data.ranges[minimum_angle]
        if minimum_distance == float('inf'):
            ## The robot can't locate a nearest object, so it halts
            self.change_velocity(0, 0)
        elif minimum_distance > dist:
            ## The robot is too far away from the object, moves closer.
            kp_speed = 0.3
            kp_angle = 0.02
            error_speed = minimum_distance
            error_angle = 0
            if minimum_angle <= 180:
                ## The object is on the left, turn left (error, ang mom > 0)
                error_angle = minimum_angle
            else:
                ## The object is on the right, turn right (error, ang mom > 0)
                error_angle = minimum_angle - 360
            if delta <= minimum_angle <= (360-delta):
                ## the robot needs to rotate to face the object
                self.change_velocity(0, error_angle*kp_angle)
            else:
                ## the robot is roughly facing the object, moves forward
                self.change_velocity(error_speed*kp_speed, error_angle*kp_angle)
        else:
            ## The robot is within 'dist' of the object, so it can stop
            self.change_velocity(0, 0)
    def run(self):
        rospy.spin()


def main():
    node = FollowPerson()
    node.run()

if __name__ == "__main__":
    main()
