#!/usr/bin/env python
import rospy
import time
from njllrd.proj2.srv import *
from njllrd.proj2.msg import *
from std_msgs.msg import String

flag = False

def controller():
    rospy.init_node('controller')
    rospy.Subscriber('user_input', 'std_msgs/String', handle_user_input)
    
    connect_points()

    
    # time.sleep(10)
    rospy.spin()

def handle_user_input():
    global flag
    flag = True


def request_endpoint():
    request = rospy.ServiceProxy('request_endpoint', request_endpoint)
    return request

def connect_points():
    global flag

    #wait for user to hit enter
    while flag == False:
        #loop

    point = request_endpoint()
    flag = False

    while flag == False:
        #loop

    #wait for user to hit enter
    point2 = request_endpoint()

    #publish waypoints to robot_interface
    print point
    print point2


if __name__ == "__main__":
    controller()
