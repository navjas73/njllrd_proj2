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
        
        pointi = point()
        
        pointi.x = point1.endpoint.x
        pointi.y = point1.endpoint.y
        pointi.z = point1.endpoint.z

        

        pointj = point()
        pointj.x = point2.endpoint.x
        pointj.y = point2.endpoint.y
        pointj.z = point2.endpoint.z

        points.points.append(pointj)
        points.points.append(pointi)
        
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

        #first_point = numpy.array([0,0,0])
        #second_point = numpy.array([.07,-.07,0])

        #first_point_rot = numpy.dot(R,first_point)
        #second_point_rot = numpy.dot(R,second_point)

        #first_point_rot_trans = first_point_rot + pointc
        #second_point_rot_trans = second_point_rot + pointc

        #points.points.append(make_point_from_array(first_point_rot_trans))
        #points.points.append(make_point_from_array(second_point_rot_trans))
        
        

        print "pointc"
        print pointc

        request = rospy.ServiceProxy('connect_waypoints', connect_waypoints)
        
        stroke_request = waypoints()
        #Try to draw an A
        data = a()
        for stroke in data[:-1]: # Get all but the last, which is the point to end on
            print "stroke"
            print stroke
            new_stroke = numpy.array([])
            first_point = True
            for orig_point in stroke:
                print "original point"
                print orig_point
                print "scaled point"
                new_point = .01*orig_point
                print new_point
                new_point = numpy.dot(R,new_point)
                print "new point"
                print new_point
                
                if first_point == True:
                    new_stroke = new_point
                    first_point = False
                else:
                    new_stroke = numpy.vstack((new_stroke, new_point))
    
            for new_stroke_point in new_stroke:
                
                new_stroke_point = new_stroke_point+pointc
                
                stroke_request.points.append(make_point_from_array(new_stroke_point))
            print "stroke_request"     
            print stroke_request       
            output = request(stroke_request)
    
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


def a():
	s_1 = numpy.array([[3,5,0],[7,5,0]])
	s_2 = numpy.array([[1,1,0],[5,9,0],[9,1,0]])
	p_end = numpy.array([9,1,0])
	return s_1,s_2,p_end

def b():
	s_1 = numpy.array([[1,5,0],[5,5,0]])
	s_2 = numpy.array([[6,1,0],[1,1,0],[1,5,0],[1,9,0],[5,9,0],[5,5,0],[6,5,0],[6,1,0]])
	p_end = numpy.array([6,1,0])
	return s_1,s_2,p_end
	
def c():
	s_1 = numpy.array([[8,9,0],[1,9,0],[1,1,0],[8,1,0]])
	p_end = numpy.array([8,1,0])
	return s_1,p_end

def d():
	s_1 = numpy.array([[8,2,0],[1,1,0],[1,9,0],[8,8,0]])
	p_end = numpy.array([8,1,0])
	return s_1,p_end

def e():
	s_1 = numpy.array([[1,5,0],[6,5,0]])
	s_2 = numpy.array([[6,9,0],[1,9,0],[1,5,0],[1,1,0],[6,1,0]])
	p_end = numpy.array([6,1,0])
	return s_1,s_2,p_end
	
def f():
	s_1 = numpy.array([[1,5,0],[6,5,0]])
	s_2 = numpy.array([[6,9,0],[1,9,0],[1,5,0],[1,1,0]])
	p_end = numpy.array([6,1,0])
	return s_1,s_2,p_end

def g():
	s_1 = numpy.array([[9,9,0],[1,9,0],[1,1,0],[9,1,0],[9,5,0],[5,5,0]])
	p_end = numpy.array([9,1,0])
	return s_1,p_end

def h():
	s_1 = numpy.array([[1,1,0],[1,9,0]])
	s_2 = numpy.array([[1,5,0],[9,5,0]])
	s_3 = numpy.array([[9,9,0],[9,1,0]])
	p_end = numpy.array([9,1,0])
	return s_1,s_2,s_3,p_end
	
def i():
	s_1 = numpy.array([[1,9,0],[5,9,0],[9,9,0]])
	s_2 = numpy.array([[5,9,0],[5,1,0]])
	s_3 = numpy.array([[1,1,0],[5,1,0],[9,1,0]])
	p_end = numpy.array([9,1,0])
	return s_1,s_2,s_3,p_end

def j():
	s_1 = numpy.array([[1,9,0],[5,9,0],[9,9,0]])
	s_2 = numpy.array([[5,9,0],[5,1,0]])
	s_3 = numpy.array([[1,1,0],[5,1,0]])
	p_end = numpy.array([9,1,0])
	return s_1,s_2,s_3,p_end

def k():
	s_1 = numpy.array([[1,1,0],[1,5,0],[1,9,0]])
	s_2 = numpy.array([[7,9,0],[1,5,0],[7,1,0]])
	p_end = numpy.array([7,1,0])
	return s_1,s_2,p_end
	







if __name__ == "__main__":
    controller()
