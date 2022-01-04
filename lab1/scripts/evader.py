#!/usr/bin/env python

from ackermann_msgs import msg
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

state = None
ignition = False
seq = 0
run = False

pub = rospy.Publisher('/drive', msg.AckermannDriveStamped,queue_size=1)


def callback(data):

    global state,ignition,seq,run
    new_state = state
    if run:
        if not ignition:
            ignition = True
            state = "forward"
            driver = msg.AckermannDriveStamped()
            driver.drive.speed = 2.0
            seq += 1
            driver.header.seq = seq
            rospy.sleep(1)
            pub.publish(driver)
            
            

        front = data.ranges[540]
        left = min(data.ranges[630:810])
        right = min(data.ranges[270:470])
        back = min(data.ranges[0:120] + data.ranges[960:1080])
        b_right = min(data.ranges[135:270])
        b_left = min(data.ranges[810:935])

        f_limit_lower = 0.6
        f_limit_upper = 1.5
        limit_lower = 0.5
        b_limit_lower = 0.6

        if front > f_limit_lower:
            if state == "reverse":
                if front > f_limit_upper:
                    new_state = "forward"
                else:
                    if back < b_limit_lower:
                        new_state = "forward"
                    else:
                        new_state = "reverse"
                    
            else:
                if left < limit_lower:
                    new_state = "left"
                elif right < limit_lower:
                    new_state = "right"
                else:
                    new_state = "forward"
        else:
            if back < b_limit_lower:
                new_state = "forward"
            else:
                new_state = "reverse"


        # print(ignition,seq,state, new_state,front)

        if state != new_state:
            state = new_state
            driver = msg.AckermannDriveStamped()
            seq += 1
            driver.header.seq = seq
            if state == "forward":
                driver.drive.speed = 2.0
                driver.drive.steering_angle = 0.0
            elif state == "left":
                driver.drive.speed = 2.0
                driver.drive.steering_angle = -0.3
            elif state == "right":
                driver.drive.speed = 2.0
                driver.drive.steering_angle = 0.3
            elif state == "reverse":
                driver.drive.speed = -2.0
                if b_left > b_right:
                    driver.drive.steering_angle = -0.3
                else:
                    driver.drive.steering_angle = 0.3
            
        
            pub.publish(driver)
        

def callback2(data):
    global run,seq
    if data.data == 'e':
        run = not run
        driver = msg.AckermannDriveStamped()
        seq += 1
        driver.header.seq = seq
        if not run:
            driver.drive.speed = 0.0
        else:
            driver.drive.speed = 2.0
        pub.publish(driver)
    
def evader():
    rospy.init_node('evader_drive', anonymous=True)
    rospy.Subscriber("key",String,callback2,buff_size=1)
    rospy.Subscriber("scan",LaserScan,callback,buff_size=1000)
    rospy.spin()

if __name__ == '__main__':
    try:
        evader()
    except rospy.ROSInterruptException:
        pass
