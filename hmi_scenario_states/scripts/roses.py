#!/usr/bin/python
import rospy
import smach
import smach_ros
import tf
import sys
import copy

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
            outcomes=['succeeded','failed'],
            output_keys=['reset_rotation'])

        rospy.Timer(rospy.Duration(0.05), self.broadcast_tf)
        self.br = tf.TransformBroadcaster()
        self.NX = 4
        self.NY = 5
        self.roses = self.fill_roses()
        self.current_rose = ""
        rospy.sleep(1)
            
    def execute(self, userdata):
        if len(self.roses) > 0:
            #self.current_rose = self.roses.pop(random.randint(0,len(self.roses)-1))
            self.current_rose = self.roses.pop()
            rospy.loginfo("selected %s", self.current_rose)
            #sss.say(["selected " + self.current_rose], False)
            rospy.sleep(0.5)
            userdata.reset_rotation = True
            return "succeeded"
        else:
            rospy.logwarn("No more roses, please fill up")
            sss.say(["No more roses, please fill up"],False)
            self.roses
            return "failed"

    def broadcast_tf(self, event):
        if self.current_rose != "":
            self.br.sendTransform(
                    (0,0,0),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     event.current_real,
                     "current_rose_fixed",
                     self.current_rose)

    def fill_roses(self):
        roses = []
        for side in ["left","right"]:
            for x in reversed(range(self.NX)):
                for y in range(self.NY):
                    roses.append("rose_" + side + "_" + str(x) + "-" + str(y))
        return roses



class RotateRose(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['succeeded','failed'],
            input_keys=['reset_rotation'],
            output_keys=['reset_rotation'])

        rospy.Timer(rospy.Duration(0.05), self.broadcast_tf)
        self.br = tf.TransformBroadcaster()
        self.angle_offset = 0
        self.max_angle = 90.0/180.0*math.pi
        self.direction = 1
        self.angle_step = 30.0/180.0*math.pi*self.direction
        rospy.sleep(1)
            
    def execute(self, userdata):
        #reset angel for second rose
        if userdata.reset_rotation:
            self.angle_offset = 0   
            userdata.reset_rotation = False
        else:
            if self.angle_offset > self.max_angle:
                # reset angle_offset
                self.angle_offset = 0
                return "failed"
            else:
                self.angle_offset += self.angle_step
        sss.say(["angle offset is " + str(self.angle_offset)])
        return "succeeded"

    def broadcast_tf(self, event):
        self.br.sendTransform(
                (0, 0, 0),
                 tf.transformations.quaternion_from_euler(0, 0, self.angle_offset),
                 event.current_real,
                 "current_rose",
                 "current_rose_fixed")


## -- main script
class GraspRose(smach.State):
    def __init__(self, side = "right"):
        smach.State.__init__(self, 
            outcomes=['succeeded','failed'],
            output_keys=['reset_rotation'])

        # initialize tf listener
        self.listener = tf.TransformListener()
        
        ### Create a handle for the Planning Scene Interface
        self.psi = PlanningSceneInterface()
        
        ### Create a handle for the Move Group Commander
        self.mgc = MoveGroupCommander("arm_" + side)
        
        self.side = side
        
        rospy.sleep(1)
        
    def execute(self, userdata):
        rospy.loginfo("Grasping rose...")
        #sss.wait_for_input()

        if not self.plan_and_execute():
            userdata.reset_rotation = False
            return "failed"
        
        return "succeeded"
           


    def plan_and_execute(self):
        ### Set next (virtual) start state
        start_state = RobotState()
        (pre_grasp_config, error_code) = sss.compose_trajectory("arm_" + self.side,"pre_grasp")
        if error_code != 0:
            rospy.logerr("unable to parse pre_grasp configuration")
            return False
        start_state.joint_state.name = pre_grasp_config.joint_names
        start_state.joint_state.position = pre_grasp_config.points[0].positions
        start_state.is_diff = True
        self.mgc.set_start_state(start_state)

        ### Plan Approach
        approach_pose_offset = PoseStamped()
        approach_pose_offset.header.frame_id = "current_rose"
        approach_pose_offset.header.stamp = rospy.Time(0)
        approach_pose_offset.pose.position.x = -0.1
        approach_pose_offset.pose.orientation.w = 1
        approach_pose = self.listener.transformPose("odom_combined", approach_pose_offset)
        
        (traj_approach,frac_approach) = self.mgc.compute_cartesian_path([approach_pose.pose], 0.01, 2, True)
        if not (frac_approach == 1.0):
            rospy.logerr("Unable to plan approach trajectory")
            sss.say(["no approach trajectory: skipping rose"])
            return False

        ### Set next (virtual) start state
        traj_approach_endpoint = traj_approach.joint_trajectory.points[-1]
        start_state = RobotState()
        start_state.joint_state.name = traj_approach.joint_trajectory.joint_names
        start_state.joint_state.position = traj_approach_endpoint.positions
        start_state.is_diff = True
        self.mgc.set_start_state(start_state)

        ### Plan Grasp
        grasp_pose_offset = PoseStamped()
        grasp_pose_offset.header.frame_id = "current_rose"
        grasp_pose_offset.header.stamp = rospy.Time(0)
        grasp_pose_offset.pose.orientation.w = 1
        grasp_pose = self.listener.transformPose("odom_combined", grasp_pose_offset)
        (traj_grasp,frac_grasp) = self.mgc.compute_cartesian_path([grasp_pose.pose], 0.01, 2, True)
        if not (frac_grasp == 1.0):
            rospy.logerr("Unable to plan grasp trajectory")
            sss.say(["no grasp trajectory: skipping rose"])
            return False

        ### Set next (virtual) start state
        traj_grasp_endpoint = traj_grasp.joint_trajectory.points[-1]
        start_state = RobotState()
        start_state.joint_state.name = traj_grasp.joint_trajectory.joint_names
        start_state.joint_state.position = traj_grasp_endpoint.positions
        start_state.is_diff = True
        self.mgc.set_start_state(start_state)

        ### Plan Lift
        lift_pose_offset = PoseStamped()
        lift_pose_offset.header.frame_id = "current_rose"
        lift_pose_offset.header.stamp = rospy.Time(0)
        lift_pose_offset.pose.position.z = 0.1
        lift_pose_offset.pose.orientation.w = 1
        lift_pose = self.listener.transformPose("odom_combined", lift_pose_offset)

        (traj_lift,frac_lift) = self.mgc.compute_cartesian_path([lift_pose.pose], 0.01, 2, True)
        if not (frac_lift == 1.0):
            rospy.logerr("Unable to plan lift trajectory")
            sss.say(["no lift trajectory: skipping rose"])
            return False


        if not (frac_approach == 1.0 and frac_grasp == 1.0 and frac_lift == 1.0):
            rospy.logerr("Unable to plan whole grasping trajectory")
            sss.say(["skipping rose"])
            return False
        else:
            sss.say(["grasping rose"], False)

            # fix trajectories to stop at the end
            traj_approach.joint_trajectory.points[-1].velocities = [0]*7
            traj_grasp.joint_trajectory.points[-1].velocities = [0]*7
            traj_lift.joint_trajectory.points[-1].velocities = [0]*7
            
            # fix trajectories to be slower
            speed_factor = 2
            for i in range(len(traj_approach.joint_trajectory.points)):
                traj_approach.joint_trajectory.points[i].time_from_start *= speed_factor
            for i in range(len(traj_grasp.joint_trajectory.points)):
                traj_grasp.joint_trajectory.points[i].time_from_start *= speed_factor
            for i in range(len(traj_lift.joint_trajectory.points)):
                traj_lift.joint_trajectory.points[i].time_from_start *= speed_factor

            ### execute
            sss.move("arm_" + self.side, "pre_grasp")
            #sss.move("gripper_" + self.side, "open")
            self.mgc.execute(traj_approach)
            self.mgc.execute(traj_grasp)
            #sss.move("gripper_" + self.side, "close")
            self.mgc.execute(traj_lift)
            rospy.sleep(0.5)
            sss.move("arm_" + self.side, "pre_grasp")
        return True

## -- State Machine 

class Roses(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,
            outcomes=['finished','failed'])
        
        self.userdata.reset_rotation = True
        
        with self:

            smach.StateMachine.add('SELECT_NEXT_ROSE',SelectCurrentRose(),
                transitions={'succeeded':'ROTATE_ROSE',
                    'failed':'failed'})

            smach.StateMachine.add('ROTATE_ROSE',RotateRose(),
                transitions={'succeeded':'GRASP_ROSE_RIGHT',
                    'failed':'SELECT_NEXT_ROSE'})

            smach.StateMachine.add('GRASP_ROSE_RIGHT',GraspRose("right"),
                transitions={'succeeded':'SELECT_NEXT_ROSE',
                    'failed':'ROTATE_ROSE'})

            #smach.StateMachine.add('GRASP_ROSE_LEFT',GraspRose("left"),
            #    transitions={'succeeded':'ROTATE_ROSE',
            #        'failed':'ROTATE_ROSE'})
                    















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
