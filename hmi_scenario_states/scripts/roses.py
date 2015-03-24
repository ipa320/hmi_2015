#!/usr/bin/python
import rospy
import smach
import smach_ros
import tf
import sys

import random

from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import RobotState

from simple_script_server import *
sss = simple_script_server()

## -- Initiation
class SelectCurrentRose(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['succeeded','failed'])

        rospy.Timer(rospy.Duration(0.1), self.broadcast_tf)
        self.br = tf.TransformBroadcaster()
        self.NX = 4
        self.NY = 5
        self.roses = self.fill_roses()
        self.current_rose = ""
        rospy.sleep(1)
            
    def execute(self, userdata):
        if len(self.roses) > 0:
            #self.current_rose = "rose_right_0-3"
            #self.current_rose = self.roses.pop(random.randint(0,len(self.roses)-1))
            self.current_rose = self.roses.pop()
            rospy.loginfo("selected %s", self.current_rose)
            #sss.say(["selected " + self.current_rose], False)
            rospy.sleep(0.5)
            return "succeeded"
        else:
            rospy.logwarn("No more roses, please fill up")
            self.roses
            return "failed"

    def broadcast_tf(self, event):
        if self.current_rose != "":
            self.br.sendTransform(
                    (0,0,0),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     event.current_real,
                     "current_rose",
                     self.current_rose)

    def fill_roses(self):
        roses = []
        for side in ["left","right"]:
            for x in range(self.NX):
                for y in range(self.NY):
                    roses.append("rose_" + side + "_" + str(x) + "-" + str(y))
        return roses


## -- main script
class GraspRose(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['succeeded','failed'])

        # initialize tf listener
        self.listener = tf.TransformListener()
        
        ### Create a handle for the Planning Scene Interface
        self.psi = PlanningSceneInterface()
        
        ### Create a handle for the Move Group Commander
        self.mgc = MoveGroupCommander("arm_right")
        
        rospy.sleep(1)
        
    def execute(self, userdata):
        rospy.loginfo("Grasping rose...")
        #sss.wait_for_input()

        ### Set next (virtual) start state
        start_state = RobotState()
        (pre_grasp_config, error_code) = sss.compose_trajectory("arm_right","pre_grasp")
        if error_code != 0:
            rospy.logerr("unable to parse pre_grasp configuration")
            return "failed"
        start_state.joint_state.name = pre_grasp_config.joint_names
        start_state.joint_state.position = pre_grasp_config.points[0].positions
        start_state.is_diff = True

        ### Plan Approach

        # transform rose into arm_7_link
        try:
            (trans, rot) = self.listener.lookupTransform("odom_combined","current_rose",rospy.Time(0))
            goal_pose = Pose()
            goal_pose.position.x = trans[0]
            goal_pose.position.y = trans[1]
            goal_pose.position.z = trans[2]
            goal_pose.orientation.x = rot[0]
            goal_pose.orientation.y = rot[1]
            goal_pose.orientation.z = rot[2]
            goal_pose.orientation.w = rot[3]
        except Exception, e:
            rospy.logerr("could get transform from base_link to current_rose. Exception: %s", str(e))
            return "failed"
        self.mgc.set_start_state(start_state)
        (traj_approach,frac_approach) = self.mgc.compute_cartesian_path([goal_pose], 0.01, 2, True)


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
        self.mgc.set_start_state(start_state)
        (traj_lift,frac_lift) = self.mgc.compute_cartesian_path([goal_pose], 0.01, 2, True)



        # fix trajectories to stop at the end
        traj_approach.joint_trajectory.points[-1].velocities = [0]*7
        traj_lift.joint_trajectory.points[-1].velocities = [0]*7
        
        # fix trajectories to be slower
        #for i in range(len(traj_approach.joint_trajectory.points)):
        #    traj_approach.joint_trajectory.points[i].time_from_start *= 2
        #for i in range(len(traj_lift.joint_trajectory.points)):
        #    traj_lift.joint_trajectory.points[i].time_from_start *= 2

        #print frac_approach
        #print traj_approach
        #print frac_lift
        #print traj_lift


        if not (frac_approach == 1.0 and frac_lift == 1.0):
            rospy.logerr("Unable to plan whole grasping trajectory")
            sss.say(["skipping rose"])
            return "failed"
        else:
            sss.say(["grasping rose"], False)
            sss.move("arm_right", "pre_grasp")
        #    sss.move("gripper_right", "open")
            self.mgc.execute(traj_approach)
        #    sss.move("gripper_right", "close")
            self.mgc.execute(traj_lift)
            rospy.sleep(0.5)
            sss.move("arm_right", "pre_grasp")

        return "succeeded"


## -- State Machine 

class Roses(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,
            outcomes=['finished','failed'])
        with self:

            smach.StateMachine.add('SELECT_CURRENT_ROSE',SelectCurrentRose(),
                transitions={'succeeded':'GRASP_ROSE',
                    'failed':'failed'})

            smach.StateMachine.add('GRASP_ROSE',GraspRose(),
                transitions={'succeeded':'SELECT_CURRENT_ROSE',
                    'failed':'SELECT_CURRENT_ROSE'})















class SM(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,outcomes=['ended'])
        with self:
            smach.StateMachine.add('STATE',Roses(),
                transitions={'finished':'ended',
                    'failed':'ended'})

if __name__=='__main__':
    rospy.init_node('cob_introduction')
    sm = SM()
    sis = smach_ros.IntrospectionServer('SM', sm, 'SM')
    sis.start()
    outcome = sm.execute()
    rospy.spin()
    sis.stop()
