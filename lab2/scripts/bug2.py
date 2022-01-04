#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
import numpy as np
from numpy.linalg import norm
from nav_msgs.msg import Odometry
import math
from tf import transformations
import time

def myhook():
  print("--------------------\ndestination reached!\n--------------------")

rospy.on_shutdown(myhook)

# global variables
position = Point()
yaw = 0
ranges = None
start = Point(-8.0,-2.0,0)
end = Point(4.5,9.0,0)
state = 1
turn_dir = "right"
# state 0 - wall follow, state 1 - line follow

loop = 0
state_time = 0

def odom(data):
    global position, yaw
    position = data.pose.pose.position
    quat = (
        data.pose.pose.orientation.x,
        data.pose.pose.orientation.y,
        data.pose.pose.orientation.z,
        data.pose.pose.orientation.w)
    yaw = math.degrees(transformations.euler_from_quaternion(quat)[2])

def laser(data):
    global ranges
    ranges = {
        'right':  min(data.ranges[:72]),
        'fright': min(data.ranges[72:144]),
        'front':  min(data.ranges[144:216]),
        'fleft':  min(data.ranges[216:288]),
        'left':   min(data.ranges[288:]),
    }

def distance_to_line(a):
    global start, end
    n = math.fabs((end.y - start.y) * a.x - (end.x - start.x) * a.y + (end.x * start.y) - (end.y * start.x))
    d = math.sqrt(pow(end.y - start.y, 2) + pow(end.x - start.x, 2))
    return n/d

def distance_to_end():
    global end,position
    distance = math.hypot(end.x-position.x, end.y-position.y)
    return distance

def polar_angle(a,b):
    return math.degrees(math.atan((b.y-a.y)/(b.x-a.x)))

yaw_end = polar_angle(start,end)

def main():
    global state,ranges,position,loop,state_time,turn_dir,yaw_end
    rospy.init_node('bug2', anonymous=True)
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        rospy.Subscriber("odom",Odometry,odom)
        rospy.Subscriber("base_scan",LaserScan,laser)
        dis_pos_line = distance_to_line(position)

        print(dis_pos_line,distance_to_end())

        if ranges == None: 
            continue

        if distance_to_end() < 0.5:
            rospy.signal_shutdown("destination reached!")
        

        if state == 1: # line follow
            if ranges['front'] > 0.1 and ranges['front'] < 0.5:
                state = 0
                state_time = 0
        
        elif state == 0: # wall follow
            if state_time > 5 and dis_pos_line < 0.1:
                state = 1
                state_time = 0
                # if turn_dir == "left":
                #     turn_dir = "right"
                # elif turn_dir == "right":
                #     turn_dir = "left"
                
        loop += 1
        if loop == 20:
            state_time = state_time + 1
            loop = 0

        move = Twist()

        if state == 0:
            if ranges["front"] < 0.5 or ranges["fleft"] < 0.4 or ranges["fright"] < 0.4:
                if turn_dir == "left":
                    move.angular.z = 0.6
                elif turn_dir == "right":
                    move.angular.z = -0.6
            
            elif ranges["left"] < 0.5 or ranges["right"] < 0.5:
                move.linear.x = 0.3

            elif (ranges["left"] > 0.6 or ranges["right"] > 0.6) and (ranges["front"] > 0.7 or ranges["fleft"] > 0.7 or ranges["fright"] > 0.7):
                move.linear.x = 0.1
                if turn_dir == "left":
                    move.angular.z = -0.3
                elif turn_dir == "right":
                    move.angular.z = 0.3

        elif state == 1:
            if abs(yaw - yaw_end) > 2:
                move.angular.z = -0.2
            else:
                move.linear.x = 0.5
        
        pub.publish(move)
        rate.sleep()

if __name__ == '__main__':
    main()