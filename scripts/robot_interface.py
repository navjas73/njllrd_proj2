#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import String
import baxter_interface
import baxter_kinematics
import numpy
from njllrd.proj2.srv import *
from njllrd.proj2.msg import *

limb = None
kinematics = None
joint_names = None
tol         = None
#waypoints = None

def move_to_point(point):
	# if q_next in reachable_workspace 
	#while goal is not reached
	#get x0 from current position
	#calculate desired velocity   v_des = (x_next - x0)/time_step 
	# get pseudoinverse
	# multiply pseudoinverse by desired velocity (J^*v_des) to get joint velocities
	# set joint velocities with set_joint_velocities
	x_goal  = [point.x point.y point.z] 
    at_goal = False
    vel_mag = 0.1
	while !at_goal:
		x0   = limb.endpoint_pose()   # current pose
        x0   = x0['position']
		dist = numpy.linalg.norm(x_goal-x0)
		if dist < tol:
            at_goal = True
			break
        else
            # check if x_goal in reachable workspace
            v_des         = [(x_goal - x0)/dist*vel_mag, 0, 0, 0]    # calulate desired velocity, zero angular velocities
	        J_psuinv      = kinematics.jacobian_pseudo_inverse()
            q_dot         = j*v_des for j in J_psuinv
            joint_command = {value:key for key in joint_names for value in q_dot[key]}
            limb.set_joint_velocities(joint_command)

def move_to_initial_point(point):
    current_pose = limb.endpoint_pose()   # current pose
    new_pose     = limb.Point(point.x, point.y, point.z)
    joints       = request_kinematics(new_pose, current_pose['orientation'],'left')
    limb.move_to_joint_positions(joints)

def command_handler(data):
    initial_point = data.points[0]
    waypoints = data.points[1::]
    
    move_to_initial_point(initial_point)
    move_to_point(data.points[1])


def handle_request_pose(data):
    endpoint = limb.endpoint_pose() # add in offset for tool later
    endpoint_position = point()
    endpoint_position.x = endpoint['position'][0]
    endpoint_position.y = endpoint['position'][1]
    endpoint_position.z = endpoint['position'][2]
    
    return endpoint_position

def robot_interface():
    rospy.init_node('robot_interface')
    

    # Subscribes to waypoints from controller, sent in a set 
    rospy.Subscriber('command', waypoints, command_handler)

    # Sends message to controller when it's done
    rospy.Publisher('state', "std_msgs/String")

    # If requested, returns endpoint pose
    s = rospy.Service('request_pose', request_pose, handle_request_pose)


    global limb 
    global kinematics
    global joint_names
    global tol
    tol         = 0.05
    joint_names = ['*_s0', '*_s1', '*_e0', '*_e1', '*_w0', '*_w1', '*_w2']
    limb        = baxter_interface.Limb('left') #instantiate limb
    kinematics  = baxter_kinematics('left')
    #global waypoints
    #waypoints = waypoints()
    # time.sleep(10)
    rospy.spin()

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
    #print ikreq
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


if __name__ == "__main__":
    robot_interface()
    
