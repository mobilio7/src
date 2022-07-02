#!/usr/bin/env python

import rospy
from turtlesim.msg import Pose


def print_turtle_pos():

	rospy.init_node('prin_turtle_pos' , anonymous=False)

	rospy.Subscreiber("/turtle/pose", Pose, callback)
	rospy.spin()


def callback(data):
	rospy.loginfo("turtle pose x: %s, Y: %s", data.x, data.y)

if __name__ == ' __main__':
	print_turtle_pos()

