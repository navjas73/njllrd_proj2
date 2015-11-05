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

def sample_point():
    global joint_limits
    new_config = numpy.array([])
    for i in range(0,len(joint_limits)):
        indv_joint_rand = random.uniform(joint_limits[i,:][1],joint_limits[i,:][2])
        new_config = new_config.append(indv_joint_rand)

def RRT_handler(data):
    goal = data.startfinish.points[0]
    start = data.startfinish.points[1]

    return data.points

def build_RRT():
    rospy.init_node('build_RRT')
    
    # Subscribes to waypoints from controller, sent in a set 
    z = rospy.Service('construct_RRT', construct_RRT, RRT_handler)

    global joint_limits
    global limb 
    global kinematics
    global joint_names
    
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
    
