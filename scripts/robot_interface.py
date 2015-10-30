#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import String
from std_msgs.msg import Header
import baxter_interface
import PyKDL
from baxter_pykdl import baxter_kinematics
import numpy

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

def move_to_point(initial_point,point):
# if q_next in reachable_workspace 
#while goal is not reached
#get x0 from current position
#calculate desired velocity   v_des = (x_next - x0)/time_step 
# get pseudoinverse
# multiply pseudoinverse by desired velocity (J^*v_des) to get joint velocities
# set joint velocities with set_joint_velocities
    x_init = numpy.array([initial_point.x, initial_point.y, initial_point.z])
    x_goal  = numpy.array([point.x, point.y, point.z])
    #correct stuff for feedback
    #correct_vector = numpy.subtract(x_goal,x_init)
    correct_vector = x_goal-x_init
    correct_dist = numpy.linalg.norm(correct_vector)
    correct_vector = correct_vector/correct_dist
    print "x_init"
    print x_init
    print "x_goal"
    print x_goal 
    at_goal = False
    vel_mag = 0.02
    kp = 0.5

    #uncomment when you don't want to recalculate position every time
    #x0   = x_init
    
    time_initial = rospy.get_time();
    deltaT  =  0;
    while not at_goal:
        #recalculating position every time
        #comment when set position once
        x0   = limb.endpoint_pose()   # current pose
        x0orientation = x0['orientation']
        x0   = x0['position']
        x0rotmax = quaternion_to_rotation(x0orientation[0],x0orientation[1],x0orientation[2],x0orientation[3])
        # offset vector. Add rotated offset vector to x0 to get desired end effector position 
        offset_vector = numpy.array([0,0,tool_length])
        rotated_offset = numpy.dot(x0rotmax,offset_vector)

        x0 = numpy.array([x0.x+rotated_offset[0,0], x0.y+rotated_offset[0,1], x0.z+rotated_offset[0,2]])

        '''print"x0"
        print x0
        print "x_goal"
        print x_goal'''

        #uncomment when ready to try feedback stuff
        deltaT = rospy.get_time() - time_initial;
        correct_x = correct_vector*vel_mag*deltaT+x_init
        error = x0 - correct_x
        error = error*kp
        
        #dist = numpy.linalg.norm(numpy.subtract(x_goal,x0))
        dist = numpy.linalg.norm(x_goal-x0)
        if dist < tol:
            at_goal = True
            print "within tolerance"
            limb.exit_control_mode()
            break
        else:
            # check if x_goal in reachable workspace

            #uncomment for feedback stuff
            v_des         = (x_goal-x0)/dist*vel_mag - error 

            #v_des = (x_goal-x0)/dist*vel_mag
            v_des = numpy.append(v_des, [0,0,0])  # calculate desired velocity, zero angular velocities
            J_psuinv      = kinematics.jacobian_pseudo_inverse()
            print "vdes"
            print v_des
            '''print "jpsu"
            print J_psuinv'''
            q_dot = numpy.dot(J_psuinv,v_des)
            #q_dot = J_psuinv*v_des
            q_dot = q_dot.tolist()
            q_dot = q_dot[0]
            '''print "qdot"
            print q_dot'''

            #I don't think this did what we wanted it to
            # joint_command = {key:value for key in joint_names for value in q_dot}

            joint_command = dict(zip(joint_names,q_dot))
            print "joint_command"
            print joint_command
            limb.set_joint_velocities(joint_command)
            '''print "joint velocities"
            print limb.joint_velocities()'''
            #did this to try to set the rate of setting commands... didn't work
            sleep(0.002)
    return True
'''def move_to_initial_point(point):
    current_pose = limb.endpoint_pose()   # current pose
    new_pose     = limb.Point(point.x, point.y, point.z)
    print "current_pose"
    print current_pose['orientation']
    print "new_pose"
    print new_pose
    joints       = request_kinematics(new_pose, current_pose['orientation'],'left')
    print "joints"
    print joints
    limb.move_to_joint_positions(joints)
    print "moved to initial point"'''

def command_handler(data):
    i = 0
    for point in data.points.points:
        if (i is not 0):
            x = move_to_point(data.points.points[i-1],point)
            print "move from point: "
            print data.points.points[i-1]
            print "to point: "
            print point
        i = i+1
    print "end of command handler"
    return True


def handle_request_endpoint(data):
    endpoint = limb.endpoint_pose() # add in offset for tool later
    endpoint_position = point()
    q = endpoint['orientation']
    #endpoint_rotation = PyKDL.Rotation.Quaternion(q[0],q[1],q[2],q[3])
    '''print "endpoint_rotation"
    print endpoint_rotation'''
    endpoint_position.x = endpoint['position'][0]
    endpoint_position.y = endpoint['position'][1]
    endpoint_position.z = endpoint['position'][2]
    '''print endpoint_position
    print "our rotation"'''
    our_rotation = quaternion_to_rotation(q[0],q[1],q[2],q[3])
    '''print our_rotation'''
    offset_vector = numpy.array([0,0,tool_length])
    rotated_offset = numpy.dot(our_rotation,offset_vector)
    '''print "rotated offset"
    print rotated_offset'''
    endpoint_position.x = endpoint_position.x + rotated_offset[0,0]
    #print rotated_offset[0,0]
    endpoint_position.y = endpoint_position.y + rotated_offset[0,1]
    endpoint_position.z = endpoint_position.z + rotated_offset[0,2]

    endpoint_position
    '''print "offset_vector"
    print offset_vector'''
    return endpoint_position


def quaternion_to_rotation(qx,qy,qz,qw):
    rotation_matrix = numpy.matrix([[1-2*qy**2-2*qz**2, 2*qx*qy-2*qz*qw, 2*qx*qz+2*qy*qw],[2*qx*qy+2*qz*qw, 1-2*qx**2-2*qz**2, 2*qy*qz-2*qx*qw],[2*qx*qz-2*qy*qw, 2*qy*qz+2*qx*qw, 1-2*qx**2-2*qy**2]])
 
    return rotation_matrix



def request_kinematics(position, quaternion, side):
    # rospy.init_node("rsdk_ik_service_client")
    ns    = "ExternalTools/" + side + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr   = Header(stamp=rospy.Time.now(), frame_id='base')
    poses = {
        'right': PoseStamped(
            header = hdr,
            pose   = Pose(position,quaternion)
            ),
        'left': PoseStamped(
            header = hdr,
            pose   = Pose(position,quaternion)
            ),
    }
    ikreq.pose_stamp.append(poses[side])
    print ikreq
    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 1
    if (resp.isValid[0]):
        #print("SUCCESS - Valid Joint Solution Found:")
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        return limb_joints
    else:
        print("INVALID POSE - No Valid Joint Solution Found.")
    return False

def robot_interface():
    rospy.init_node('robot_interface')
    
    # Subscribes to waypoints from controller, sent in a set 
    z = rospy.Service('connect_waypoints', connect_waypoints, command_handler)

    # Sends message to controller when it's done
    rospy.Publisher('state', String, queue_size=10)
    
    # If requested, returns endpoint pose
    s = rospy.Service('request_endpoint', request_endpoint, handle_request_endpoint)


    global limb 
    global kinematics
    global joint_names
    global tol
    tol         = 0.01
    joint_names = ['left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2']
    limb        = baxter_interface.Limb('left') #instantiate limb
    kinematics  = baxter_kinematics('left')
    global points
    points = waypoints()
    rospy.spin()


if __name__ == "__main__":
    robot_interface()
    
