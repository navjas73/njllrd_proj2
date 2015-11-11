#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import String
from std_msgs.msg import Header
import baxter_interface
import PyKDL
from baxter_pykdl import baxter_kinematics
import numpy
import random
import scipy.spatial
from collision_checker.srv import CheckCollision

from njllrd_proj2.srv import *
from njllrd_proj2.msg import *
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from time import sleep

limb = None
kinematics = None
joint_names = None
tol         = None
points = None
tool_length = .15
joint_limits = None
nodes_RRT = numpy.array([])
edges_RRT = numpy.array([[-1,-1]])
nodes_BIRRT = numpy.array([])
edges_BIRRT = numpy.array([[-1,-1]])
stepSize = 0.02

def sample_point():
    global joint_limits
    q_rand = numpy.array([])
    for i in range(0,len(joint_limits)):
        indv_joint_rand = random.uniform(joint_limits[i,:][0],joint_limits[i,:][1])
        q_rand = numpy.append(q_rand,indv_joint_rand)
    return q_rand

def nearest_neighbor(q_rand, nodes):
    tree = scipy.spatial.KDTree(nodes[:,0:7])    # configurations need to be row numpy arrays
    d, i = tree.query(q_rand)          # returns ("distance to nearest neighbors", index of nearest neighbor --- we want index)
    q_near = tree.data[i]
    return d,i,q_near

def line_to_point(step,distance,index,q_near,q_rand, add_to_tree, nodes, edges):
    num_points = int(distance/step)
    '''print "distance"
    print distance
    print "num_points"
    print num_points '''
    q_prev = q_near 
    parent = index
    end = 0
    #count = 0
    for i in range(1,num_points+1):
        q_next = (q_rand - q_near)/distance*stepSize*i + q_near
        #collide = 0
        # collision check q_next
        request = rospy.ServiceProxy('/check_collision', CheckCollision)
        rospy.wait_for_service('/check_collision')
        result = request(String('left'),q_next)  
        if not result.collision:
            #count +=1
        #if not collide:
            if add_to_tree:
                new_node = numpy.asarray([numpy.append(q_next, index)])
                nodes = numpy.concatenate((nodes,new_node), axis = 0)
                edges = numpy.concatenate((edges,numpy.array([[parent,len(nodes)-1]])),axis = 0)
                parent = len(nodes)-1 
        else:
            end = 1
    if end == 0:
        reached_end = 1
    else: 
        reached_end = 0
    #print "reached_end"
    #print reached_end
    '''print "fake distance"
    print num_points*step
    print "count"
    print count
    print "end"
    print end
    print reached_end'''
    return reached_end, nodes, edges

def determine_path(nodes, edges, start_index):
    path = numpy.array([nodes[start_index,0:7]])
    parent = nodes[start_index,7]
    while parent > -1:
        path = numpy.concatenate((numpy.asarray([nodes[parent,0:7]]),path),axis=0)
        parent = nodes[parent,7]
    return path

def BIRRT_handler(data):
    global nodes_BIRRT
    global edges_BIRRT
    global nodes_RRT
    global edges_RRT
    global stepSize
    RRT_index = 0
    BIRRT_index = 0
    goal = numpy.asarray(data.goal)
    start = numpy.asarray(data.start)
    nodes_RRT = numpy.array([numpy.append(start,-1)])
    nodes_BIRRT = numpy.array([numpy.append(goal,-1)])
    reached_goal = 0
    reached_connect = 0
    start_goal, nodes_RRT, edges_RRT = line_to_point(stepSize, numpy.linalg.norm(goal-start), 0, start, goal, 0, nodes_RRT, edges_RRT)
    if start_goal:
        reached_goal = 1
        combined_path = numpy.array([start,goal])
    else:
        # generate random q
        print "starting loop"
        while not reached_connect:
            q_rand = sample_point()
            dist, index, q_near= nearest_neighbor(q_rand, nodes_RRT)
            reached_rand, nodes_RRT, edges_RRT = line_to_point(stepSize, dist, index, q_near, q_rand, 1, nodes_RRT, edges_RRT)
            print "reached_rand_RRT"
            print reached_rand
            dist_connect, index_connect, q_connect = nearest_neighbor(nodes_RRT[-1,0:7], nodes_BIRRT)
            print "nodes_RRT"
            print nodes_RRT
            reached_connect, nodes_RRT, edges_RRT = line_to_point(stepSize, dist_connect, index_connect, nodes_RRT[-1,0:7], q_connect, 0, nodes_RRT, edges_RRT)
            print "nodes_RRT"
            print nodes_RRT
            if reached_connect:
                RRT_index = -1
                BIRRT_index = index_connect
                break

            q_rand = sample_point()
            dist, index, q_near= nearest_neighbor(q_rand, nodes_BIRRT)
            reached_rand, nodes_BIRRT, edges_BIRRT = line_to_point(stepSize, dist, index, q_near, q_rand, 1, nodes_BIRRT, edges_BIRRT)
            print "reached_rand_BIRRT"
            print reached_rand
            dist_connect, index_connect, q_connect= nearest_neighbor(nodes_BIRRT[-1,0:7], nodes_RRT)
            reached_connect, nodes_BIRRT, edges_BIRRT = line_to_point(stepSize, dist_connect, index_connect, nodes_BIRRT[-1,0:7], q_connect, 0, nodes_BIRRT, edges_BIRRT)
            if reached_connect:
                RRT_index = index_connect
                BIRRT_index = -1
                break 
        path_RRT = determine_path(nodes_RRT, edges_RRT, RRT_index)
        path_BIRRT = determine_path(nodes_BIRRT, edges_BIRRT, BIRRT_index)
        print "start"
        print start
        print "goal"
        print goal
        print "nodes RRT"
        print nodes_RRT
        print "nodes BIRRT"
        print nodes_BIRRT
        print "edges RRT"
        print edges_RRT
        print "edges BIRRT"
        print edges_BIRRT
        print "path RRT"
        print path_RRT
        print "path_BIRRT"
        print path_BIRRT
        combined_path = numpy.concatenate((path_RRT, path_BIRRT[::-1]), axis = 0)
        print "new path"
        print combined_path
    new_path = numpy.array([])
    for i in combined_path:
        path_point = single_config()
        path_point.config = i
        new_path = numpy.append(new_path, path_point)
    return new_path

def RRT_handler(data):
    global nodes_RRT
    global edges_RRT
    global stepSize
    goal = numpy.asarray(data.goal)
    start = numpy.asarray(data.start)
    nodes_RRT = numpy.array([numpy.append(start,-1)])
    reached_goal = 0
    start_goal, nodes_RRT, edges_RRT = line_to_point(stepSize, numpy.linalg.norm(goal-start), 0, start, goal, 0, nodes_RRT, edges_RRT)
    print start_goal
    if start_goal:
        reached_goal = 1
        path = numpy.array([start,goal])
    else:
        # generate random q
        print "starting loop"
        while not reached_goal:

            q_rand = sample_point()
            #print "qrand"
            #print q_rand
            # get q_near
            dist, index, q_near= nearest_neighbor(q_rand, nodes_RRT)
            #print "qnear"
            #print q_near
            # get point some distance from q_near
            #q_new = (q_rand - q_near)/numpy.linalg.norm(q_rand - q_near) * stepSize + q_near
            #print "qnew"
            #print q_new
            reached_rand, nodes_RRT, edges_RRT = line_to_point(stepSize, dist, index, q_near, q_rand, 1, nodes_RRT, edges_RRT)
            #print "nodes"
            #print nodes[0]
            #print nodes[-1]
            print "reached_rand"
            print reached_rand
            goal_dist = numpy.linalg.norm(goal-nodes_RRT[-1,0:7])
            #node_length_test = len(nodes)
            reached_goal, nodes_RRT, edges_RRT = line_to_point(stepSize,goal_dist,len(nodes_RRT)-1, nodes_RRT[-1,0:7], goal, 0, nodes_RRT, edges_RRT)
            print reached_goal
            if reached_goal:
                new_node = numpy.asarray([numpy.append(goal, len(nodes_RRT)-1)])
                nodes_RRT = numpy.concatenate((nodes_RRT,new_node), axis = 0)
                edges_RRT = numpy.concatenate((edges_RRT,numpy.array([[len(nodes_RRT)-2,len(nodes_RRT)-1]])),axis = 0)
                print "start"
                print start
                print "goal"
                print goal
                print "nodes"
                print nodes_RRT
                print len(nodes_RRT)
                print "edges"
                print edges_RRT
        # full numpy array path
        path = determine_path(nodes_RRT, edges_RRT,-1)
        print path 

    # Smooth here
    if len(path) > 3:
        path, nodes_RRT, edges_RRT = smooth_path(path,nodes_RRT,edges_RRT)
    print "final smoothed path"
    print path 
    new_path = numpy.array([])
    #convert numpy path to service array
    for i in path:
        path_point = single_config()
        path_point.config = i
        new_path = numpy.append(new_path, path_point)


        #print "NEW TEST"
        #print goal
        #print "nodes"
        #print nodes
        #print "edges"
        #print edges
        #print nodes[node_length_test-1]
        #print nodes[node_length_test]
        #print nodes[-1]
        # check collision for q_new
        # check for collision between q_new and q_goal
        # if collision between q_new and q_goal but no collision for q_new, add vertex and add edge
        # else if no collision between q_new and q_goal and no collision for q_new
        # add vertex and edge. Then add goal as vertex and edge. Then exit  for loop

        # get path from edges and vertices 
        
        # add or dont add to tree (edge and node)
        # check if can see q_goal 
    return new_path
def smooth_path(path, nodes, edges):
    ''' roslaunch baxter_gazebo baxter_world.launch 
    rosrun baxter_tools enable_robot.py -e
    rosrun baxter_examples joint_position_keyboard.py 
    llimb.set_joint_positions({'left_w0': 1, 'left_w1': -1.2, 'left_w2': 1.8721031098678509e-06, 'left_e0': 0.2, 'left_e1': 2, 'left_s0': -1.5, 'left_s1': -0.3})
    '''
    
    i = 0
    smoothed_path = path
    while i < len(smoothed_path)-2:
        n = len(smoothed_path)
        point = smoothed_path[i]
        test_index = random.randint(i+2,n-1)
        test_point = smoothed_path[test_index]
        success, nodes, edges = line_to_point(stepSize,numpy.linalg.norm(test_point-point),0,point, test_point,0,nodes,edges)
        if success:
            #smoothed_path = smoothed_path[0:i] 
            smoothed_path = numpy.concatenate((smoothed_path[0:i+1],smoothed_path[test_index:n]),axis=0)
            print "test smoothed path"
            print smoothed_path 
            #smoothed_path.concatenate(smoothed_path[test_index:n-1])
        i = i+1


    return smoothed_path, nodes, edges





def build_RRT():
    rospy.init_node('build_RRT')
    
    # Subscribes to waypoints from controller, sent in a set 
    z = rospy.Service('construct_RRT', construct_RRT, RRT_handler)

    a = rospy.Service('construct_BIRRT', construct_BIRRT, BIRRT_handler)

    global joint_limits
    global limb 
    global kinematics
    global joint_names
    stepSize = 0.017    # ~1 degree in radians
    # Left limb
    joint_names = ['left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2']
    limb        = baxter_interface.Limb('left') #instantiate limb
    kinematics  = baxter_kinematics('left')
    joint_limits = numpy.array([[-2.461, .890],[-2.147,1.047],[-3.028,3.028],[-.052,2.618],[-3.059,3.059],[-1.571,2.094],[-3.059,3.059]])
    max_joint_speeds = numpy.array([2.0,2.0,2.0,2.0,4.0,4.0,4.0])

    # Right limb
    #joint_names = ['right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2']
    #limb = baxter_interface.Limb('right')
    #kinematics = baxter_kinematics('right')
    #joint_limits = numpy.array([[-2.461, .890],[-2.147,1.047],[-3.028,3.028],[-.052,2.618],[-3.059,3.059],[-1.571,2.094],[-3.059,3.059]])
    #max_joint_speeds = numpy.array([2.0,2.0,2.0,2.0,4.0,4.0,4.0])


    global points
    points = waypoints()
    rospy.spin()


if __name__ == "__main__":
    build_RRT()
    
