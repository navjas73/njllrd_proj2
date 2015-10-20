#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import String
import baxter_interface

limb = None

def x_next(data):
	# if q_next in reachable_workspace 
	#v_des = (x_next - x0)/time_step 
	#while goal is not reached
	#get x0 from current position
	#calculate desired velocity
	# get pseudoinverse
	# multiply pseudoinverse by desired velocity (J^*v_des) to get joint velocities
	# set joint velocities with set_joint_velocities
	x_goal = data.data 
	while !goal:
		current_pose = limb.endpoint_pose()
        current_pose = current_pose['position']
		dist = numpy.linalg.norm(data.data-current_pose)
		if dist < 0.05:
			break
	

def controller():
    rospy.init_node('controller')
    # rospy.wait_for_service('move_robot')
    rospy.Subscriber('command', list, x_next) # dont know what "list" should really be
    # rospy.Subscriber('state', state, read_state)
    global limb 
    limb = baxter_interface.Limb('right') #instantiate limb
    # time.sleep(10)
    rospy.spin()

if __name__ == "__main__":
    controller()
    
