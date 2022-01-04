#!/usr/bin/env python
from operator import pos
import rospy
from geometry_msgs.msg import Twist
import numpy as np
from nav_msgs.msg import Odometry
import math
from tf import transformations

goalx = 4.5
goaly = 9.0

shift = (-8.5,9.5)
end = (-int(math.floor(goaly - shift[1])),int(math.floor(goalx-shift[0])))

print(end)

map = [0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,
       0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,
       0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
       1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
       0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
       0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0,
       0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0,
       0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,1,0,
       0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,1,1,
       0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,1,1,
       0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,1,1,
       0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,0,
       0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,
       0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,
       0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
       0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,
       0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,0,
       0,0,0,0,0,0,0,0,1,1,1,0,0,1,1,1,1,0,
       0,0,0,0,0,0,0,1,1,1,0,0,0,1,1,1,1,0,
       0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,1]

map = np.array(map).reshape(20,18)

dist = np.zeros(map.shape)
for i in range(map.shape[0]):
    for j in range(map.shape[1]):
        dist[i,j] = np.sqrt((i-end[0])**2+(j-end[1])**2)

prev = np.zeros(map.shape)
# 1 is up
# 2 is right
# 3 is down 
# 4 is left

trav = np.zeros(map.shape)

queue = [(12,1)]
finished = []

def sortq(queue):
    tots = []
    for i in queue:
        tots.append(dist[i]+trav[i])
    tots,queue = zip(*sorted(zip(tots,queue)))
    return list(queue)

while end not in queue:

    now = queue[0]
    del queue[0]

    if now[0]-1 >= 0 and map[now[0]-1,now[1]] != 1:
        x = (now[0]-1,now[1])
        if x not in finished:
            if x not in queue:
                prev[x] = 3
                trav[x] = trav[now] + 1
                queue.append(x)
            elif trav[x] > trav[now] + 1:
                prev[x] = 3
                trav[x] = trav[now] + 1

    if now[0]+1 < 20 and map[now[0]+1,now[1]] != 1:
        x = (now[0]+1,now[1])
        if x not in finished:
            if x not in queue:
                prev[x] = 1
                trav[x] = trav[now] + 1
                queue.append(x)
            elif trav[x] > trav[now] + 1:
                prev[x] = 1
                trav[x] = trav[now] + 1

    if now[1]-1 >= 0 and map[now[0],now[1]-1] != 1:
        x = (now[0],now[1]-1)
        if x not in finished:
            if x not in queue:
                prev[x] = 2
                trav[x] = trav[now] + 1
                queue.append(x)
            elif trav[x] > trav[now] + 1:
                prev[x] = 2
                trav[x] = trav[now] + 1

    if now[1]+1 < 18 and map[now[0],now[1]+1] != 1:
        x = (now[0],now[1]+1)
        if x not in finished:
            if x not in queue:
                prev[x] = 4
                trav[x] = trav[now] + 1
                queue.append(x)
            elif trav[x] > trav[now] + 1:
                prev[x] = 4
                trav[x] = trav[now] + 1

    finished.append(now)
    queue = sortq(queue)


path = []
now = end
while now != (12,1):
    path.append(now)
    if prev[now]   == 1:
        now = (now[0]-1,now[1])
    elif prev[now] == 2:
        now = (now[0],now[1]+1)
    elif prev[now] == 3:
        now = (now[0]+1,now[1])
    elif prev[now] == 4:
        now = (now[0],now[1]-1)
path.append(now)
path.reverse()


def odom(data):
    global position, yaw
    position = data.pose.pose.position
    quat = (
        data.pose.pose.orientation.x,
        data.pose.pose.orientation.y,
        data.pose.pose.orientation.z,
        data.pose.pose.orientation.w)
    yaw = math.degrees(transformations.euler_from_quaternion(quat)[2])

def polar_angle(a,b):
    return math.degrees(math.atan2(b[1]-a.y,b[0]-a.x))

position = None
yaw = None

for i in range(len(path)):
    path[i] = (path[i][1]+shift[0],-path[i][0]+shift[1])


def distance(end,position):
    dis = math.hypot(end[0]-position.x, end[1]-position.y)
    return dis

def myhook():
  print("Final destination reached")

final_path = []
final_path.append(path[0])
for i in range(1,len(path)-1):
    if abs(path[i-1][0] - path[i][0]) + abs(path[i+1][0] - path[i][0]) == 1:
        final_path.append(path[i])
final_path.append(path[-1])

print(path)
print(final_path)


def main():
    global final_path,position, yaw
    rospy.init_node('astar', anonymous=True)
    pub = rospy.Publisher("cmd_vel", Twist,queue_size=1000)
    rate = rospy.Rate(1000)
    
    while position == None:
        rospy.Subscriber("odom",Odometry,odom)
        rate.sleep()

    mode_180 = True

    while not rospy.is_shutdown() and final_path:
        rospy.Subscriber("odom",Odometry,odom)
        # print(position.x,position.y)
        
        dirc = polar_angle(position,final_path[0])
        # print(dirc,yaw,final_path[0])
        move = Twist()
        distt = distance(final_path[0],position)

        if dirc*yaw < -30625:
            if yaw > 0:
                move.angular.z = 0.1
            else:
                move.angular.z = -0.1
        elif abs(dirc - yaw) > 4:    
            if dirc - yaw > 0:
                move.angular.z = 0.3 + 0.02*abs(dirc - yaw)
            else:
                move.angular.z = -(0.3 + 0.02*abs(dirc - yaw))
        elif distt > 0.1:
            move.linear.x = 0.1 + 2*distt
        elif distt < 0.1:
            del final_path[0]
            if not final_path:
                rospy.on_shutdown(myhook)
                continue
            print("next destination: "+str(final_path[0]))
        

        pub.publish(move)
        rate.sleep()




    

if __name__ == '__main__':
    main()

