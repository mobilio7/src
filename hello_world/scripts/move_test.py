#!/usr/bin/env python

import rospy
import math
import numpy as np

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


SCAN_STEP = 0.5 # deg
VIEW_ANGLE = 45.0 # deg

P = int((VIEW_ANGLE / 2) / SCAN_STEP)
# -P, ..., 0, ... P

TIME_STEP = 0.01
LIN_SPEED = 0.15
ANG_SPEED = 0.5

BRAKE_DIST = 0.40 # distance at which braking starts
STOP_DIST = 0.2 # distance at which robot is estimated to stop

KP_AVOID, KD_AVOID = 1.0, 1.0



BRAKE_ACC = LIN_SPEED ** 2 / 2 / (BRAKE_DIST - STOP_DIST) # braking deceleration
SCAN_STEP_RAD = SCAN_STEP * math.pi / 180

def sgn(x):
    return 1.0 if x >= 0.0 else -1.0

def take_nonzero(arr):
    return arr[np.nonzero(arr)]

def ind2rad(ind, N):
    # N : number of probing angles
    # return : rad value ( -pi ~ +pi )
    ang = 2 * math.pi * ind / (N-1)
    return ang if ang <= math.pi else ang - 2*math.pi

class LidarGet:
    def __init__(self):
        self.last_dists = [float('inf')] * 4 
        # Subscriber
        rospy.Subscriber('/scan', LaserScan, self.callback)
        # Publisher
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.forward = True
        self.brake = False
        #self.turn90 = False
        self.avoid = False

        self.avoid_radius = float('inf')

        self.min_ang = 0.0
        self.min_dist = float('inf')

        self.time_lapse = 0.0

        self.v = 0.0
        self.omega = 0.0

        self.prev_dist_err = 0.0

        self.is_close = True

        

        rospy.Timer(rospy.Duration(TIME_STEP), self.give_cmd)

    def give_cmd(self, event):
     
        vel_msg = Twist()

        #if not self.avoid:
        if self.forward:
            print("FORWARD")
            print("forward dist = {:.2f}, closest dist = {:.2f}".format(self.last_dists[1], self.min_dist))
            vel_msg.linear.x = LIN_SPEED
            vel_msg.angular.z = 0.0
            #self.avoid = self.last_dists[1] < STOP_DIST
            if self.last_dists[1] < BRAKE_DIST or self.min_dist < BRAKE_DIST:
                self.forward = False
                self.brake = True
                self.brake_needed = True
                self.time_lapse = 0.0
                if self.last_dists[1] < STOP_DIST:
                    self.brake_needed = False
        elif self.brake:
            print("BRAKE")
            vel_msg.linear.x = LIN_SPEED - BRAKE_ACC * self.time_lapse
            
            if not self.brake_needed:
                if self.last_dists[1] < STOP_DIST:
                    vel_msg.linear.x = -LIN_SPEED * 0.3       
                else:
                    vel_msg.linear.x = 0.0  
                    self.brake = False
                    self.avoid = True
                    self.time_lapse = 0.0
                    #self.ang_sign = sgn(abs(self.min_ang) - math.pi/2)

            print("VEL = ", vel_msg.linear.x)
            if vel_msg.linear.x < 0.0 :
                vel_msg.linear.x = 0.0
                self.brake = False
                #self.turn90 = True
                self.avoid = True
                self.time_lapse = 0.0
                #self.ang_sign = sgn(abs(self.min_ang) - math.pi/2)
            else:
                vel_msg.angular.z = 0.0
                self.time_lapse += TIME_STEP
        #elif self.turn90:
        #    print("TURN90")
        #    vel_msg.angular.z = -ANG_SPEED if self.min_ang > 0 else +ANG_SPEED
        #    print("min deg = ", self.min_ang * 180.0 / math.pi)
        #    #if abs(abs(self.min_ang) - math.pi/2) < 2 * SCAN_STEP_RAD:
        #    if sgn(abs(self.min_ang) - math.pi/2) != self.ang_sign:
        #        vel_msg.linear.x = 0.0
        #        vel_msg.angular.z = 0.0
        #        self.turn90 = False
        #        self.avoid = True
        #        self.v = LIN_SPEED / 2
        #        self.omega = 0.0
        #    print("ang vel = ", vel_msg.angular.z)
        #    #left_dist = self.last_dists[2]
        #    #if left_dist > STOP_DIST:
        #    #    vel_msg.linear.x = 0.0
        #    #    vel_msg.angular.z = -ANG_SPEED if self.first_turn else +ANG_SPEED
        #    #else:
        #    #    self.first_turn = False
        #    #    vel_msg.linear.x = LIN_SPEED
        #    #    vel_msg.angular.z = 0.0
        elif self.avoid:
            print("AVOID")

            rel_ang = abs(self.min_ang) - math.pi/2

            if abs(self.min_ang) < math.pi / 3:
                vel_msg.linear.x = -LIN_SPEED * 1.5
                vel_msg.angular.z = +ANG_SPEED * sgn(self.min_ang)
            elif abs(self.min_ang) > (math.pi - math.pi / 3):
                vel_msg.linear.x = +LIN_SPEED * 1.5
                vel_msg.angular.z = -ANG_SPEED * sgn(self.min_ang)
            else:
                vel_msg.linear.x = +LIN_SPEED * 1.5 * sgn(rel_ang)
                vel_msg.angular.z = +ANG_SPEED * sgn(rel_ang) * sgn(self.min_ang)

            if self.min_dist > BRAKE_DIST * 1.2:
                self.avoid = False
                self.forward = True

            ## If heading the obstacle -> turn
            #if rel_ang < -(10.0) * math.pi/180:
            #    vel_msg.linear.x = -LIN_SPEED 
            #    vel_msg.angular.z = -ANG_SPEED * sgn(self.min_ang)
            #else:
            #    vel_msg.linear.x = 0.0
            #    vel_msg.angular.z = 0.0

          

   #         print("self.is_close = ", self.is_close)
   #         print("eta = {:.2f}, rel_ang = {:.2f}".format(eta, rel_ang * 180/math.pi))

    #        if self.is_close:
    #            linear_speed = max(0.0, LIN_SPEED * (1 + (0.1 - 1) * eta))
    #            angular_speed = max(0.0, ANG_SPEED * eta)
    #            vel_msg.linear.x = linear_speed            
    #            vel_msg.angular.z = angular_speed * sgn(self.min_ang) * rel_ang     
    #            if eta > 1.0:
    #                if rel_ang < -math.pi/6:
    #                    self.is_close = False
    #                else:
    #                    vel_msg.angular.z = angular_speed * sgn(self.min_ang)

            # too close & point inside => slow speed, turn outside
            # too close & point outside => go straight
            # too far & point inside =
 

            print("linear = {:.2f}, angular = {:.2f}".format(vel_msg.linear.x, vel_msg.angular.z))
            print()

            


        self.cmd_vel_pub.publish(vel_msg)

    def callback(self, data):
        distances = np.array([dist for dist in data.ranges])
        N = len(distances)

        # Rear distances
        rear_ind = N//2  
        rear_dists = distances[rear_ind-P : rear_ind+P+1]
        rear_dists = take_nonzero(rear_dists)

        # Front distances
        front_ind = N-1
        front_dists = np.concatenate([distances[front_ind-P:], distances[:P]])
        front_dists = take_nonzero(front_dists)

        # Left distances
        left_ind = N//4
        left_dists = distances[left_ind-P : left_ind+P+1]
        left_dists = take_nonzero(left_dists)

        # Right distances
        right_ind = 3*N // 4
        right_dists = distances[right_ind-P : right_ind+P+1]
        right_dists = take_nonzero(right_dists)

        # Show 4-directional distances
        min_dists = []
        for ind, dists in enumerate([rear_dists, front_dists, left_dists, right_dists]):
            min_dists.append(self.last_dists[ind] if len(dists)==0 else dists.min())
        self.last_dists = min_dists
     
        # Find minimum distances and its angular position (w.r.t robot)
        nz = distances.nonzero()
        min_ind = nz[0][distances[nz].argmin()]
        self.min_ang = ind2rad(min_ind, N)
        self.min_dist = distances[min_ind]
        

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('lidar_get_node')
    node = LidarGet()
    node.main()
