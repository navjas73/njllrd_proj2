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
nodes = numpy.array([])
edges = numpy.array([[-1,-1]])
stepSize = 0.02

def sample_point():
    global joint_limits
    q_rand = numpy.array([])
    for i in range(0,len(joint_limits)):
        indv_joint_rand = random.uniform(joint_limits[i,:][0],joint_limits[i,:][1])
        q_rand = numpy.append(q_rand,indv_joint_rand)
    return q_rand

def nearest_neighbor(q_rand):
    global nodes
    tree = scipy.spatial.KDTree(nodes[:,0:7])    # configurations need to be row numpy arrays
    d, i = tree.query(q_rand)          # returns ("distance to nearest neighbors", index of nearest neighbor --- we want index)
    q_near = tree.data[i]
    return d,i,q_near

def line_to_point(step,distance,index,q_near,q_rand, add_to_tree):
    global nodes
    global edges
    num_points = int(distance/step)
    '''print "distance"
    print distance
    print "num_points"
    print num_points '''
    q_prev = q_near 
    parent = index
    end = 0
    for i in range(1,num_points+1):
        q_next = (q_rand - q_near)/distance*stepSize*i + q_near
        collide = 0
        # collision check q_next
        request = rospy.ServiceProxy('/check_collision', CheckCollision)
        rospy.wait_for_service('/check_collision')
        result = request(String('left'),q_next)  
        if not result.collision:
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
    #print "fake distance"
    #print num_points*step
    return reached_end 

def determine_path():
    global nodes
    global edges
    path = numpy.array([nodes[-1,0:7]])
    parent = nodes[-1,7]
    while parent > -1:
        path = numpy.concatenate((numpy.asarray([nodes[parent,0:7]]),path),axis=0)
        parent = nodes[parent,7]
        print parent
    print "path"
    print path
    return path



def RRT_handler(data):
    global nodes
    global edges
    global stepSize
    goal = numpy.asarray(data.goal)
    start = numpy.asarray(data.start)
    nodes = numpy.array([numpy.append(start,-1)])
    reached_goal = 0
    # generate random q
    print "starting loop"
    while not reached_goal:

        q_rand = sample_point()
        #print "qrand"
        #print q_rand
        # get q_near
        dist, index, q_near = nearest_neighbor(q_rand)
        #print "qnear"
        #print q_near
        # get point some distance from q_near
        #q_new = (q_rand - q_near)/numpy.linalg.norm(q_rand - q_near) * stepSize + q_near
        #print "qnew"
        #print q_new
        reached_rand = line_to_point(stepSize, dist, index, q_near, q_rand, 1)
        #print "nodes"
        #print nodes[0]
        #print nodes[-1]
        print "reached_rand"
        print reached_rand
        goal_dist = numpy.linalg.norm(goal-nodes[-1,0:7])
        #node_length_test = len(nodes)
        reached_goal = line_to_point(stepSize,goal_dist,len(nodes)-1, nodes[-1,0:7], goal, 0)
        print reached_goal
        if reached_goal:
            new_node = numpy.asarray([numpy.append(goal, len(nodes)-1)])
            nodes = numpy.concatenate((nodes,new_node), axis = 0)
            edges = numpy.concatenate((edges,numpy.array([[len(nodes)-2,len(nodes)-1]])),axis = 0)
            print "start"
            print start
            print "goal"
            print goal
            print "nodes"
            print nodes
            print len(nodes)
            print "edges"
            print edges
    path = determine_path()


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

    return []

def build_RRT():
    rospy.init_node('build_RRT')
    
    # Subscribes to waypoints from controller, sent in a set 
    z = rospy.Service('construct_RRT', construct_RRT, RRT_handler)

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
    
