#!/usr/bin/env python
import rospy
import time
import math
import numpy
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


        pointa = numpy.array([point1.endpoint.x, point1.endpoint.y, point1.endpoint.z])
        pointb = numpy.array([point2.endpoint.x, point2.endpoint.y, point2.endpoint.z])
        pointc = numpy.array([point3.endpoint.x, point3.endpoint.y, point3.endpoint.z])
        
        plane_vec = numpy.cross(pointa-pointb, pointb-pointc)

        plane_normal = plane_vec/numpy.linalg.norm(plane_vec)

        R = make_rotation_matrix(plane_normal)



        # connect <0,0,0> and <.2,.2,0>
        #physical placement of 3 points on plane 
        # pt2            pt1
        # pt3

        first_point = numpy.array([0,0,0])
        second_point = numpy.array([.07,-.07,0])

        first_point_rot = numpy.dot(R,first_point)
        second_point_rot = numpy.dot(R,second_point)

        first_point_rot_trans = first_point_rot + pointc
        second_point_rot_trans = second_point_rot + pointc

        points.points.append(make_point_from_array(first_point_rot_trans))
        points.points.append(make_point_from_array(second_point_rot_trans))
        request = rospy.ServiceProxy('connect_waypoints', connect_waypoints)
        output = request(points)
    elif rospy.get_param('/mode')=="RRT":
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
        rospy.wait_for_service('construct_RRT')
        request = rospy.ServiceProxy('construct_RRT', construct_RRT)
        output = request(points)  
        print output     


    # time.sleep(10)
    rospy.spin()

def make_point(data):
    the_point = data
    the_real_point = point()
    the_real_point.x = the_point.endpoint.x
    the_real_point.y = the_point.endpoint.y
    the_real_point.z = the_point.endpoint.z
    return the_real_point

def make_point_from_array(data):
    the_point = data
    the_real_point = point()
    the_real_point.x = the_point[0]
    the_real_point.y = the_point[1]
    the_real_point.z = the_point[2]
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
    
def make_rotation_matrix(plane_normal):
    z = numpy.array([0, 0, 1])
    k = numpy.cross(z,plane_normal)
    kx = k[0]
    ky = k[1]
    kz = k[2]

    theta = math.acos(numpy.dot(z,plane_normal))
    v = 1-math.cos(theta)
    c = math.cos(theta)
    s = math.sin(theta)

    R = numpy.array([[kx**2*v+c, kx*ky*v - kz*s, kx*kz*v + ky*s],[kx*kz*v + kz*s, ky**2*v + c, ky*kz*v - kx*s],[kx*kz*v - ky*s, ky*kz*v + kx*s, kz**2 * v +c]])
    return R




if __name__ == "__main__":
    controller()
