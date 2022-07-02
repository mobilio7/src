#!/usr/bin/env python

import rospy
from omo_r1mini_bringup.srv import Battery

bat_status = rospy.ServiceProxy('/battery_status',Battery)
bat_status()

if __name__ == '__main__':
    try:
        battery()
    except rospy.ROSInterruptException:
        pass