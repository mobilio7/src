#!/usr/bin/env python

#import rospy
#from geometry_msgs.msg import Twist
#
#def move_turtle1():
#    rospy.init_node('omo_r1mini_node', anonymous=True)
#    vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
#  #  vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
#    vel_ref = Twist()
#
#    rate = rospy.Rate(10) # 10hz
#
#    while not rospy.is_shutdown():
#        vel_ref.linear.x = 0.4
#        vel_ref.angular.z = 2
#    	rospy.loginfo(vel_ref)
#        vel_pub.publish(vel_ref)
#        rate.sleep()
#
#if __name__ == '__main__':
#    try:
#        move_turtle1()
#    except rospy.ROSInterruptException:
#        pass

import sys
import rospy
import math
from time import sleep
#from omo_packet_handler import PacketHandler

#from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, Point, Vector3, Quaternion

WAY_POINTS = [(0.0, 0.0), (1.0, 0.0)]

#WAY_POINTS = [(0,0), (1.0, 0.3), (2.0, -0.3)]
#WAY_POINTS = [(0.0, 0.0), (0.0, 0.3), (1.0, 0.3), (1.0, 0.0), (2.0, 0.0)]
STOP_DIST = 0.01
TARGET_LINEAR_SPEED = 0.25 # step = 0.05
TARGET_ANGULAR_SPEED = 0.25 # step = 0.10
TIME_STEP = 0.01



DIST_STEP = TARGET_LINEAR_SPEED * TIME_STEP
ANGLE_STEP = TARGET_ANGULAR_SPEED * TIME_STEP


def angle_wrapped(ang):
    # Output : angle (-180 ~ +180)
    pass



def get_trans_durations(waypoints, time_step=TIME_STEP, lin_speed=TARGET_LINEAR_SPEED):
    N = len(waypoints)
    durations = []
    for i in range(N-1):
        p0, p1 = waypoints[i], waypoints[i+1]
        dx, dy = p1[0] - p0[0], p1[1] - p0[1]
        dist = math.sqrt(dx**2 + dy**2)
        duration = dist / TARGET_LINEAR_SPEED
        durations.append(duration)
    return durations


def get_rotate_durations(waypoints, time_step=TIME_STEP, ang_speed=TARGET_ANGULAR_SPEED):
    # NOTE : plus if direction is +, minus if directions is -
    N = len(waypoints)
    durations = []
    prev_theta = 0.0
    for i in range(N-1):
        p0, p1 = waypoints[i], waypoints[i+1]
        dx, dy = p1[0] - p0[0], p1[1] - p0[1]
        theta = math.atan2(dy, dx)
        time_diff = (theta - prev_theta) / TARGET_ANGULAR_SPEED
        durations.append(time_diff)
        prev_theta = theta
    return durations
        

class SimpleFollowNode:
    def __init__(self, waypoints): 
   
        #self.x, self.y = 0, 0
        #self.theta = 0

        self.target_ind = 0 
        self.goal_reached = False

        self.waypoints = waypoints
        self.trans_durations = get_trans_durations(self.waypoints)
        self.rotate_durations = get_rotate_durations(self.waypoints)

        self.trans_time = 0.0
        self.rotate_time = 0.0 
   
        self.rotating = True
        self.theta = 0.0

        self.x, self.y, self.theta = 0.0, 0.0, 0.0
        self.base_x, self.base_y, self.base_theta = 0.0, 0.0, 0.0

        # Publisher
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Subscriber
        rospy.Subscriber('/odom', Odometry, self.callback)

        rospy.Timer(rospy.Duration(TIME_STEP), self.give_cmd)
     #   self.odom_pose.timestamp = rospy.Time.now()
     #   self.odom_pose.pre_timestamp = rospy.Time.now()

    def callback(self, data):
        pose = data.pose.pose
        #pose.position.x, pose.orientation.z
        self.x, self.y = pose.position.x, pose.position.y
        self.theta = pose.orientation.z
        #if self.is_start:
        #    self.is_start = False
        #    self.x_bias, self.y_bias, self.theta_bias = pose.position.x, pose.position.y, pose.orientation.z
        #    self.x, self.y, self.theta = 0.0, 0.0, 0.0
        #else:
        #    self.x, self.y, self.theta = pose.position.x, pose.position.y, pose.orientation.z
        #    self.x -= self.x_bias
        #    self.y -= self.y_bias
        #    self.theta -= self.theta_bias
        

    def give_cmd(self, event):

        #print("self.target_ind = ", self.target_ind)

        #print("self.trans_durations = ", self.trans_durations)
        #print("self.trans_time = ", self.trans_time)
        #print("self.rotate_time = ", self.rotate_time)
       

        self.update_target()
        if self.goal_reached:
            return      

        #tar_x, tar_y = self.waypoints[self.target_ind]
        #dx, dy = tar_x - self.x, tar_y - self.y
        #tar_angle = math.atan2(dy, dx) - self.theta
        #tar_vx, tar_vy = TARGET_SPEED * math.cos(tar_angle), TARGET_SPEED * math.sin(tar_angle)
       
        vel_msg = Twist()
        
        vel_msg.linear.y = 0.0
        vel_msg.linear.z = 0.0
        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0

        if self.rotating:
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = TARGET_ANGULAR_SPEED if self.rotate_durations[self.target_ind - 1] > 0.0 else -TARGET_ANGULAR_SPEED
        else:
            vel_msg.linear.x = TARGET_LINEAR_SPEED
            vel_msg.angular.z = 0.0
        
        self.cmd_pub.publish(vel_msg)

        if self.rotating:
            self.rotate_time += TIME_STEP
        else:
            self.trans_time += TIME_STEP

    def update_target(self):
        # check goal
        if self.goal_reached:
            return
        # check start of motion
        if self.target_ind == 0:
            self.target_ind += 1
            self.rotating = True
            return
        # If transition ends, then switch to rotation
        #if not self.rotating and self.trans_time >= self.trans_durations[self.target_ind - 1]:
        i = self.target_ind
        if not self.rotating and math.hypot(self.x - self.base_x, self.y - self.base_y) >= math.hypot(self.waypoints[i-1][0] - self.waypoints[i][0], self.waypoints[i-1][1] - self.waypoints[i][1]):
            self.rotating = True
            self.trans_time = 0.0
            self.rotate_time = 0.0
            self.target_ind += 1
            if self.target_ind >= len(self.waypoints):
                self.goal_reached = True

            self.base_theta = self.theta

        # If rotation is done, then switch to transition
        #if self.rotating and self.rotate_time >= abs(self.rotate_durations[self.target_ind - 1]): 

        i = self.target_ind
        p0, p1 = self.waypoints[i-1], self.waypoints[i]
        x0, y0 = p0
        x1, y1 = p1
        dx, dy = x1 - x0, y1 - y0
        tar_theta = math.atan2(dy, dx)

        #print("tar_ind = ", self.target_ind, ", ang_diff = ", abs(self.theta * math.pi - tar_theta), ", dist = ", math.hypot(self.x - self.base_x, self.y - self.base_y))
        if not self.rotating:
            print("tar_ind = ", self.target_ind, ", dist = ", math.hypot(self.x - self.base_x, self.y - self.base_y))
        if self.rotating:
            print("rotating angle = ", (self.theta - self.base_theta) * 180.0, " deg")
        #if self.rotating and abs(self.theta * math.pi - tar_theta) <= 0.01:
        if self.rotating and self.rotate_time >= abs(self.rotate_durations[self.target_ind - 1]): 
            self.rotating = False
            self.trans_time = 0.0
            self.rotate_time = 0.0

            self.base_x, self.base_y = self.x, self.y
            
        #else:
        #    point0, point1 = self.waypoints[self.target_ind - 1], self.waypoints[self.target_ind]
        #    dx, dy = point1[0] - point0[0], point1[1] - point0[1]
        #    tar_theta = math.atan2(dy, dx)   
        #    self.cmd_pub.linear.x = 0.0
        #    self.cmd_pub.linear.y = 0.0
        #    self.cmd_pub.angular.z = TARGET_ANGULAR_SPEED if (tar_theta - self.theta > 0) else -TARGET_ANGULAR_SPEED
            
                    

    #def update_target(self):
    #    if self.goal_reached:
    #        return
    #    tar_x, tar_y = self.waypoints[self.target_ind]
    #    square_dist = (tar_x - self.x) ** 2 + (tar_y -  self.y) ** 2
    #    if square_dist <= STOP_DIST**2:
    #        if target_ind == len(self.waypoints) - 1:
    #            self.goal_reached = True
    #        else:
    #            self.target_ind += 1
        

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('simple_follow_node')
    node = SimpleFollowNode(WAY_POINTS)
    node.main()
