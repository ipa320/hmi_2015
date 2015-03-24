#!/usr/bin/env python
import rospy

import tf
from tf.transformations import *
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import RobotState
import sys

from simple_script_server import *
sss = simple_script_server()

 
### Helper function 
def gen_pose(frame_id="/base_link", pos=[0,0,0], euler=[0,0,0]):
	pose = PoseStamped()
	pose.header.frame_id = frame_id
	pose.header.stamp = rospy.Time(0)
	pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = pos
	pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = quaternion_from_euler(*euler)
	return pose

if __name__ == '__main__':
	rospy.init_node('cob_grasping_hmi')
	while rospy.get_time() == 0.0: pass
	
	# initialize tf listener
	listener = tf.TransformListener()
	rospy.sleep(1)
	
	### Create a handle for the Planning Scene Interface
	psi = PlanningSceneInterface()
	rospy.sleep(1.0)
	
	### Create a handle for the Move Group Commander
	mgc = MoveGroupCommander("arm_right")
	rospy.sleep(1.0)
	
	### Initialize simple_script_server
	sss = simple_script_server()
	
	#sss.move("arm_right", "side")
	#sss.move("gripper_right", "close")
	
	### Set next (virtual) start state
	start_state = RobotState()
	(pre_grasp_config, error_code) = sss.compose_trajectory("arm_right","pre_grasp")
	if error_code != 0:
		rospy.logerr("unable to parse pre_grasp configuration")
		sys.exit()
	start_state.joint_state.name = pre_grasp_config.joint_names
	start_state.joint_state.position = pre_grasp_config.points[0].positions
	start_state.is_diff = True
	
	### Plan Approach
	
	# transform rose into arm_7_link
	try:
		(trans, rot) = listener.lookupTransform("odom_combined","current_rose",rospy.Time(0))
		goal_pose = Pose()
		goal_pose.position.x = trans[0]
		goal_pose.position.y = trans[1]
		goal_pose.position.z = trans[2]
		goal_pose.orientation.x = rot[0]
		goal_pose.orientation.y = rot[1]
		goal_pose.orientation.z = rot[2]
		goal_pose.orientation.w = rot[3]
		print goal_pose
	except Exception, e:
		rospy.logerr("could get transform from base_link to current_rose. Exception: %s", str(e))
		sys.exit()
	mgc.set_start_state(start_state)
	(traj_approach,frac_approach) = mgc.compute_cartesian_path([goal_pose], 0.01, 4, True)
	print frac_approach
	print traj_approach
	
	### Set next (virtual) start state
	traj_approach_endpoint = traj_approach.joint_trajectory.points[-1]
	start_state = RobotState()
	#start_state.header = traj_approach.header
	#start_state.header.stamp = rospy.Time.now()
	start_state.joint_state.name = traj_approach.joint_trajectory.joint_names
	start_state.joint_state.position = traj_approach_endpoint.positions
	start_state.is_diff = True
	
	### Plan Lift
	goal_pose.position.z += 0.1
	mgc.set_start_state(start_state)
	(traj_lift,frac_lift) = mgc.compute_cartesian_path([goal_pose], 0.01, 4, True)
	print frac_lift
	print traj_lift

	if not (frac_approach == 1.0 and frac_lift == 1.0):
		rospy.logerr("Unable to plan whole grasping trajectory")
	else:
		sss.move("arm_right", "pre_grasp")
	#	sss.move("gripper_right", "open")
		mgc.execute(traj_approach)
	#	sss.move("gripper_right", "close")
		mgc.execute(traj_lift)
		sss.move("arm_right", "pre_grasp")
	
	
	print "Done"
