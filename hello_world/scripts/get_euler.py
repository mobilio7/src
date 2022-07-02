#!/usr/bin/env python

import rospy
import math
import numpy as np

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class GetEuler:
    def __init__(self):

        # Subscriber
        rospy.Subscriber('/odom', Odometry, self.callback)

        # Publisher
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        rospy.Timer(rospy.Duration(TIME_STEP), self.give_cmd)

    def callback(self, data):
        quat = data.pose.pose.orientation
        

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('get_euler_node')
    node = GetEuler()
    node.main()
