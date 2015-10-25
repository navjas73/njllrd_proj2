#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import String
from std_msgs.msg import Header
import baxter_interface
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

limb = None
kinematics = None
joint_names = None
tol         = None
points = None

def move_to_point(point):
# if q_next in reachable_workspace 
#while goal is not reached
#get x0 from current position
#calculate desired velocity   v_des = (x_next - x0)/time_step 
# get pseudoinverse
# multiply pseudoinverse by desired velocity (J^*v_des) to get joint velocities
# set joint velocities with set_joint_velocities
    x_goal  = [point.x, point.y, point.z] 
    at_goal = False
    vel_mag = 0.1
    while not at_goal:
        x0   = limb.endpoint_pose()   # current pose
        x0   = x0['position']
        x0 = [x0.x, x0.y, x0.z]
        
        dist = numpy.linalg.norm(numpy.subtract(x_goal,x0))
        if dist < tol:
            at_goal = True
            print "within tolerance"
            break
        else:
            # check if x_goal in reachable workspace
            v_des         = numpy.subtract(x_goal,x0)/dist*vel_mag  
            v_des = numpy.append(vdes, [0,0,0])  # calulate desired velocity, zero angular velocities
            J_psuinv      = kinematics.jacobian_pseudo_inverse()
            print v_des
            print J_psuinv
            q_dot         = numpy.dot(J_psuinv,v_des)
            joint_command = {value:key for key in joint_names for value in q_dot[key]}
            limb.set_joint_velocities(joint_command)

def move_to_initial_point(point):
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
    print "moved to initial point"

def command_handler(data):
    initial_point = data.points.points[0]
    points = data.points.points[1::]
    print "initial_point"
    print initial_point
    move_to_initial_point(initial_point)
    move_to_point(data.points.points[1])
    print "end of command handler"
    return True


def handle_request_endpoint(data):
    endpoint = limb.endpoint_pose() # add in offset for tool later
    endpoint_position = point()
    endpoint_position.x = endpoint['position'][0]
    endpoint_position.y = endpoint['position'][1]
    endpoint_position.z = endpoint['position'][2]
    
    return endpoint_position


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
    tol         = 0.05
    joint_names = ['*_s0', '*_s1', '*_e0', '*_e1', '*_w0', '*_w1', '*_w2']
    limb        = baxter_interface.Limb('left') #instantiate limb
    kinematics  = baxter_kinematics('left')
    global points
    points = waypoints()
    rospy.spin()


if __name__ == "__main__":
    robot_interface()
    
