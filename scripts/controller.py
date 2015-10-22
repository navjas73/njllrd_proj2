#!/usr/bin/env python
import rospy
import time
from njllrd_proj2.srv import *
from njllrd_proj2.msg import *
from std_msgs.msg import String

flag = False

def controller():
    rospy.init_node('controller')
    rospy.wait_for_service('request_endpoint')
    rospy.Subscriber('user_input', String, handle_user_input)
    
    connect_points()

    
    # time.sleep(10)
    rospy.spin()

def handle_user_input(data):
    global flag
    flag = True
    #print "got here"


def request_position():
    yes = True
    request = rospy.ServiceProxy('request_endpoint', request_endpoint)
    output = request(yes)
    return output

def connect_points():
    global flag

    #wait for user to hit enter
    while flag == False:
        x = 1
        #loop

    point = request_position()
    flag = False

    while flag == False:
        x = 1
        #loop

    #wait for user to hit enter
    point2 = request_position()

    #publish waypoints to robot_interface
    print point
    print point2

    


if __name__ == "__main__":
    controller()
