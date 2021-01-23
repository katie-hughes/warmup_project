#!/usr/bin/env python3
""" Description """
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3


from sensor_msgs.msg import LaserScan

import numpy as np


angles = np.arange(360)
buffer = 0.4
delta = 10

class FollowWall(object):
    """ description """
    def __init__(self):
        rospy.init_node("follow_wall")
        rospy.Subscriber("/scan", LaserScan, self.process_scan)
        self.twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
        r = rospy.Rate(2)
        lin = Vector3(x = 0.1)
        ang = Vector3()
        self.twist = Twist(linear=lin, angular = ang)
    def process_scan(self, data):
        print(data.ranges)
        print(len(data.ranges))
        ## len of data.ranges is 360
        minimum = np.argmin(np.array(data.ranges))
        minimum_distance = data.ranges[minimum]
        minimum_angle = angles[minimum]
        print("the minimum index is", minimum)
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
        if 90-delta <= minimum_angle <= 90+delta:
            if minimum_distance <= buffer:
                print("just go straight")
                self.twist.linear.x = 0.2
                self.twist_pub.publish(self.twist)
            else:
                print("adjust angular velocity (+z)")
                error1 = minimum_distance - buffer
                kp1 = 0.01
                self.twist.linear.x = 0.2
                self.twist.angular.z = kp1*error1
                self.twist_pub.publish(self.twist)
        else:
            print("adjust angular velocity ")
            error2 = minimum_angle - 90
            kp2 = 0.01
            self.twist.linear.x = 0.2
            self.twist.angular.z = kp2*error2
            self.twist_pub.publish(self.twist)

        ### i
            #self.twist.linear.x = 0.5
            #self.twist_pub.publish(self.twist)
    def run(self):
        rospy.spin()


def main():
    node = FollowWall()
    node.run()

if __name__ == "__main__":
    main()
