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
    

    if rospy.get_param('/mode') == "connect_points":
        point1, point2 = get_connect_points()
        
        points = waypoints()
        
        pointa = point()
        
        pointa.x = point1.endpoint.x
        pointa.y = point1.endpoint.y
        pointa.z = point1.endpoint.z

        

        pointb = point()
        pointb.x = point2.endpoint.x
        pointb.y = point2.endpoint.y
        pointb.z = point2.endpoint.z

        points.points.append(pointb)
        points.points.append(pointa)
        
        print points.points[0]
        print points.points[1]
        print points.points
        request = rospy.ServiceProxy('connect_waypoints', connect_waypoints)
        output = request(points)

    elif rospy.get_param('/mode') == "draw":
        point1, point2, point3 = get_plane_points()
        points = waypoints()

        pointa = make_point(point1)
        pointb = make_point(point2)
        pointc = make_point(point3)

        points.points.append(pointa)
        points.points.append(pointb)
        points.points.append(pointc)

        print points.points
        request = rospy.ServiceProxy('connect_waypoints', connect_waypoints)
        output = request(points)


    # time.sleep(10)
    rospy.spin()

def make_point(data):
    the_point = data
    the_real_point = point()
    the_real_point.x = the_point.endpoint.x
    the_real_point.y = the_point.endpoint.y
    the_real_point.z = the_point.endpoint.z
    return the_real_point

def handle_user_input(data):
    global flag
    flag = True
    #print "got here"


def request_position():
    yes = True
    request = rospy.ServiceProxy('request_endpoint', request_endpoint)
    output = request(yes)
    return output

def get_connect_points():
    global flag
    flag = False
    #wait for user to hit enter
    while flag == False:
        x = 1
        #loop

    point1 = request_position()
    flag = False

    while flag == False:
        x = 1
        #loop

    #wait for user to hit enter
    point2 = request_position()

    #publish waypoints to robot_interface

    return (point1, point2)

def get_plane_points():
    global flag
    flag = False

    #wait for user to hit enter
    while flag == False:
        x = 1
        #loop

    point1 = request_position()
    flag = False

    while flag == False:
        x = 1
        #loop

    #wait for user to hit enter
    point2 = request_position()
    flag = False

    while flag == False:
        x = 1
        #loop

    #wait for user to hit enter
    point3 = request_position()
    flag = False
    #publish waypoints to robot_interface
    #print (point1, point2, point3)
    return (point1, point2, point3)
    


if __name__ == "__main__":
    controller()
