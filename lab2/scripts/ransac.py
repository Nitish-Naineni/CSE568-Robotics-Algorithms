#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from math import cos,sin
import random
import numpy as np
from numpy.linalg import norm
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point,Twist
import math

pub = rospy.Publisher("ransac_vis", Marker, queue_size=10)

iters = 10
resolution = 0.05
line_rez = 0.1

def distance_between_points(a,b):
    dist = math.hypot(b[0] - a[0], b[1] - a[1])
    return dist

def slope(a):
    x1 = a[0][0]
    y1 = a[0][1]
    x2 = a[1][0]
    y2 = a[1][1]
    return (y2-y1)/(x2-x1)


def ransac(data):
    global iters,resolution,line_rez
    marks = []
    angle_increment = 0.00872664619237
    for i in range(len(data.ranges)):
        if data.intensities[i] == 1:
            r = data.ranges[i]
            if r > 0:
                theta = i*angle_increment
                marks.append(np.array([r*cos(theta),r*sin(theta)]))

    marker1 = Marker()
    marker1.header.frame_id = "/base_link"
    marker1.type = Marker.LINE_LIST
    marker1.color.r = 0
    marker1.color.g = 0
    marker1.color.b = 0
    marker1.color.a = 1
    marker1.scale.x = 0.1       
    
    if len(marks) < 2:
        print("nothing in range")
    else:
        queue = []

        while len(marks) > 2:
            inliers_best_fit = 0
            best_fit = None
            first = None
            second = None
            remove_list = [None,None]
            for i in range(iters):

                inliers = 0
                a,b = random.sample(marks, 2)

                for x in marks:
                    distance = norm(np.cross(b-a, a-x))/norm(b-a)
                    if distance < resolution:
                        inliers += 1
                
                if inliers > inliers_best_fit:
                    inliers_best_fit = inliers
                    best_fit = (a,b)

            add_line = True

            for i in range(len(marks)):
                distance = norm(np.cross(best_fit[1]-best_fit[0], best_fit[0]-marks[i]))/norm(best_fit[1]-best_fit[0])
                if distance < line_rez:
                    if first == None:
                        first = Point(x=marks[i][0],y=marks[i][1])
                        remove_list[0] = i
                    else:
                        second = Point(x=marks[i][0],y=marks[i][1])
                        remove_list[1] = i
            
            # print(first,second)
            if len(marks[:remove_list[0]]) > 2:
                queue.append(marks[:remove_list[0]])
            if len(marks[remove_list[1]:]) > 2:
                queue.append(marks[remove_list[1]:])
            
            if queue:
                marks = queue.pop()
            else:
                marks = []

            marker1.points.append(first)
            marker1.points.append(second)

    pub.publish(marker1)
    print(len(marker1.points))

        

def main():
    rospy.init_node('ransac', anonymous=True)
    rospy.Subscriber("base_scan",LaserScan,ransac,buff_size=1000)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass