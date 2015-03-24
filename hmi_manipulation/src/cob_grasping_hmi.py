#!/usr/bin/env python
import roslib; roslib.load_manifest('cob_moveit_interface')
import rospy

from tf.transformations import *
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander, PlanningSceneInterface

from cob_script_server import *
 
### Helper function 
def gen_pose(frame_id="/base_link", pos=[0,0,0], euler=[0,0,0]):
	pose = PoseStamped()
	pose.header.frame_id = frame_id
	pose.header.stamp = rospy.Time.now()
	pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = pos
	pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = quaternion_from_euler(*euler)
	return pose

if __name__ == '__main__':
	rospy.init_node('cob_grasping_hmi')
	while rospy.get_time() == 0.0: pass
	
	### Create a handle for the Planning Scene Interface
	psi = PlanningSceneInterface()
	rospy.sleep(1.0)
	
	### Create a handle for the Move Group Commander
	mgc = MoveGroupCommander("arm_right")
	rospy.sleep(1.0)
	
	### Initialize simple_script_server
	sss = simple_script_server()
	
	sss.move("arm_right", "side")
	sss.move("gripper_right", "close")
	
	### Plan Approach
	goal_pose = gen_pose(frame_id="current_rose", pos=[0.0, 0.0, 0.0], euler=[0.0, 0.0, 0.0])
	(traj_approach,frac_approach) = mgc.compute_cartesian_path([goal_pose.pose], 0.01, 4, True)
	
	### Set next (virtual) start state
	traj_approach_endpoint = traj_approach.points[-1]
	start_state = RobotState()
	start_state.header = traj_approach.header
	start_state.header.stamp = rospy.Time.now()
	start_state.joint_state.name = traj_approach.joint_names
	start_state.joint_state.position = traj_approach_endpoint.positions
	start_state.joint_state.velocity = traj_approach_endpoint.velocities
	start_state.joint_state.effort = traj_approach_endpoint.effort
	start_state.is_diff = True
	
	### Plan Lift
	goal_pose.pose.position.z += 0.1
	mgc.set_start_state(start_state)
	(traj_lift,frac_lift) = mgc.compute_cartesian_path([goal_pose.pose], 0.01, 4, True)
	
	if not (frac_approach == 1.0 && frac_lift == 1.0):
		rospy.logerr("Unable to plan whole grasping trajectory")
	else:
		sss.move("arm_right", "pre_grasp")
		sss.move("gripper_right", "open")
		mgc.execute(traj_approach)
		sss.move("gripper_right", "close")
		mgc.execute(traj_lift)
		sss.move("arm_right", "deliver")
	
	
	print "Done"
