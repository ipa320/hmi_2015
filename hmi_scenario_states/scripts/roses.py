#!/usr/bin/python
import rospy
import rospkg
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


class Init(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','failed'])

    def execute(self, userdata):

        sss.move("base","middle",mode="linear")

        handle_gripper_left = sss.move("gripper_left","home")
        handle_gripper_right = sss.move("gripper_right","home")
        handle_gripper_left.wait()
        handle_gripper_right.wait()

        handle_arm_left = sss.move("arm_left","side",False)
        handle_arm_right = sss.move("arm_right","side",False)
        handle_arm_left.wait()
        handle_arm_right.wait()

        return "succeeded"

class CheckForStop(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['stop','continue'])
        self.stop_requested = False
        rospy.Service('/hmi/stop_roses', Trigger, self.stop_cb)
            
    def execute(self, userdata):
        if self.stop_requested:
            self.stop_requested = False
            return "stop"
        else:
            return "continue"

    def stop_cb(self, req):
        self.stop_requested = True
        res = TriggerResponse()
        res.success.data = True
        return res

class Finalize(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','failed'])

    def execute(self, userdata):

        handle_gripper_left = sss.move("gripper_left","home")
        handle_gripper_right = sss.move("gripper_right","home")
        handle_gripper_left.wait()
        handle_gripper_right.wait()

        handle_arm_left = sss.move("arm_left","side",False)
        handle_arm_right = sss.move("arm_right","side",False)
        handle_arm_left.wait()
        handle_arm_right.wait()

        sss.move("base","middle",mode="linear")
        return "succeeded"


class SelectTable(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['left','right'])
        self.side = "left"

    def execute(self, userdata):
        if self.side == "left":
            self.side = "right"
        elif self.side == "right":
            self.side = "left"
        else:
            rospy.logerr("invalid side: %s", self.side)
            sys.exit()
        return self.side

class SelectCurrentRose(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['left','right','failed','empty_left','empty_right'],
            output_keys=['reset_rotation'])

        self.current_rose = ""
        rospy.Timer(rospy.Duration(0.1), self.broadcast_tf)
        self.br = tf.TransformBroadcaster()
        (self.roses_left, self.roses_right) = self.fill_roses()
        rospy.sleep(1)
            
    def execute(self, userdata):
        #print "roses_left =", self.roses_left
        #print "roses_right =", self.roses_right
        if len(self.roses_right) > 0:
            self.current_rose = self.roses_right.pop(0)
            
            # check if this rose is in first row
            xes = []
            for rose in self.roses_right:
                xes.append(rose.split("_")[2].split("-")[0])
            current_x = self.current_rose.split("_")[2].split("-")[0]
            if current_x == min(xes):
                rospy.loginfo("selected %s", self.current_rose)
                sss.say(["selected " + self.current_rose], False)
                rospy.sleep(0.5) #FIXME: wait for TF to update
                userdata.reset_rotation = True
                return "right"
            else:
                print "no more graspable roses right"
                self.roses_right = []            
        
        if len(self.roses_left) > 0:
            self.current_rose = self.roses_left.pop(0)
            
            # check if this rose is in first row
            xes = []
            for rose in self.roses_left:
                xes.append(rose.split("_")[2].split("-")[0])
            current_x = self.current_rose.split("_")[2].split("-")[0]
            if current_x == min(xes):
                rospy.loginfo("selected %s", self.current_rose)
                sss.say(["selected " + self.current_rose], False)
                rospy.sleep(0.5) #FIXME: wait for TF to update
                userdata.reset_rotation = True
                return "left"
            else:
                print "no more graspable roses left"
                self.roses_left = []

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
        roses_left = []
        roses_right = []
        roses_param = rospy.get_param("/hmi/roses")
        for rose in roses_param:
            if "left" in rose[2]:
                roses_left.append(rose[2])
            else:
                roses_right.append(rose[2])
        return roses_left, roses_right



class RotateRose(smach.State):
    def __init__(self, side):
        smach.State.__init__(self, 
            outcomes=['succeeded','failed'],
            input_keys=['reset_rotation'],
            output_keys=['reset_rotation'])

        rospy.Timer(rospy.Duration(0.1), self.broadcast_tf)
        self.br = tf.TransformBroadcaster()
        self.side = side
        self.angle_offset_roll = 0
        self.min_angle_yaw = -30.0/180.0*math.pi
        self.max_angle_yaw = 60.0/180.0*math.pi
        self.angle_step_yaw = 30.0/180.0*math.pi
        self.angle_offset_yaw = self.min_angle_yaw
        if self.side == "left":
            self.angle_offset_roll = math.pi
            self.angle_step_yaw *= -1
        elif self.side == "right":
            self.angle_offset_roll = 0
            self.angle_step_yaw *= 1
        else:
            rospy.logerr("invalid side: %s", self.side)
            sys.exit()
        
        rospy.sleep(1)
            
    def execute(self, userdata):
        #reset angel for next rose
        if userdata.reset_rotation:
            self.angle_offset_yaw = self.min_angle_yaw
            userdata.reset_rotation = False
        else:
            if self.angle_offset_yaw <= -self.max_angle_yaw or self.angle_offset_yaw >= self.max_angle_yaw:
                # reset angle_offset_yaw
                self.angle_offset_yaw = self.min_angle_yaw
                sss.say(["skipping rose"],False)
                rospy.sleep(1) # FIXME: wait for TF to update
                
                return "failed"
            else:
                self.angle_offset_yaw += self.angle_step_yaw
        #sss.say(["angle offset yaw is " + str(int(round(self.angle_offset_yaw/math.pi*180)))])
        #print "angle offset yaw is " + str(int(round(self.angle_offset_yaw/math.pi*180)))
        rospy.sleep(1) # FIXME: wait for TF to update
        
        return "succeeded"

    def broadcast_tf(self, event):
        self.br.sendTransform(
                (0, 0, 0),
                 tf.transformations.quaternion_from_euler(self.angle_offset_roll, 0, self.angle_offset_yaw),
                 event.current_real,
                 "current_rose_" + self.side,
                 "current_rose_fixed")

class SelectArm(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['left', 'right', 'failed'])

        # initialize tf listener
        self.listener = tf.TransformListener()
        
        rospy.sleep(1)
        
    def execute(self, userdata):

        try:
            (trans,rot) = self.listener.lookupTransform('/base_link', '/current_rose_fixed', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("no transformation from base_link to current_rose_fixed")
            return "failed"

        if trans[1] >= 0.0:
            return "left"
        else:
            return "right"

class GraspRose(smach.State):
    def __init__(self, side = "right"):
        smach.State.__init__(self, 
            outcomes=['succeeded','failed'],
            output_keys=['reset_rotation'])

        # initialize tf listener
        self.listener = tf.TransformListener()
        
        ### Create a handle for the Planning Scene Interface
        self.psi = PlanningSceneInterface()
        ps = PoseStamped()
        ps.header.frame_id = "table_top"
        ps.pose.orientation.w = 1
        filename = rospkg.RosPack().get_path("hmi_manipulation") + "/files/hmi_table.stl"
        self.psi.add_mesh("table", ps, filename)
        
        ### Create a handle for the Move Group Commander
        self.mgc = MoveGroupCommander("arm_" + side)
        
        self.side = side
        
        self.eef_step = 0.01
        self.jump_threshold = 4
        
        rospy.sleep(1)
        
    def execute(self, userdata):
        #rospy.loginfo("Grasping rose...")
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
        approach_pose_offset.header.frame_id = "current_rose_" + self.side
        approach_pose_offset.header.stamp = rospy.Time(0)
        approach_pose_offset.pose.position.x = -0.12
        approach_pose_offset.pose.orientation.w = 1
        try:
            approach_pose = self.listener.transformPose("odom_combined", approach_pose_offset)
        except Exception, e:
            rospy.logerr("could not transform pose. Exception: %s", str(e))
            return False
        
        (traj_approach,frac_approach) = self.mgc.compute_cartesian_path([approach_pose.pose], self.eef_step, self.jump_threshold, True)
        if not (frac_approach == 1.0):
            rospy.logerr("Unable to plan approach trajectory")
            #sss.say(["no approach trajectory: skipping rose"], False)
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
        grasp_pose_offset.header.frame_id = "current_rose_" + self.side
        grasp_pose_offset.header.stamp = rospy.Time(0)
        grasp_pose_offset.pose.orientation.w = 1
        grasp_pose = self.listener.transformPose("odom_combined", grasp_pose_offset)
        (traj_grasp,frac_grasp) = self.mgc.compute_cartesian_path([grasp_pose.pose], self.eef_step, self.jump_threshold, True)
        if not (frac_grasp == 1.0):
            rospy.logerr("Unable to plan grasp trajectory")
            #sss.say(["no grasp trajectory: skipping rose"], False)
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
        lift_pose_offset.header.frame_id = "current_rose_" + self.side
        lift_pose_offset.header.stamp = rospy.Time(0)
        if self.side == "left":
            lift_pose_offset.pose.position.z = -0.12
        elif self.side == "right":
            lift_pose_offset.pose.position.z = 0.12
        else:
            rospy.logerr("invalid side: %s", self.side)
            sys.exit()
        lift_pose_offset.pose.orientation.w = 1
        lift_pose = self.listener.transformPose("odom_combined", lift_pose_offset)

        (traj_lift,frac_lift) = self.mgc.compute_cartesian_path([lift_pose.pose], self.eef_step, self.jump_threshold, True)
        if not (frac_lift == 1.0):
            rospy.logerr("Unable to plan lift trajectory")
            #sss.say(["no lift trajectory: skipping rose"], False)
            return False


        if not (frac_approach == 1.0 and frac_grasp == 1.0 and frac_lift == 1.0):
            rospy.logerr("Unable to plan whole grasping trajectory")
            sss.say(["skipping rose"], False)
            return False
        else:
            sss.say(["grasping rose"], False)

            # fix trajectories to stop at the end
            traj_approach.joint_trajectory.points[-1].velocities = [0]*7
            traj_grasp.joint_trajectory.points[-1].velocities = [0]*7
            traj_lift.joint_trajectory.points[-1].velocities = [0]*7
            
            # fix trajectories to be slower
            speed_factor = 1
            for i in range(len(traj_approach.joint_trajectory.points)):
                traj_approach.joint_trajectory.points[i].time_from_start *= speed_factor
            for i in range(len(traj_grasp.joint_trajectory.points)):
                traj_grasp.joint_trajectory.points[i].time_from_start *= speed_factor
            for i in range(len(traj_lift.joint_trajectory.points)):
                traj_lift.joint_trajectory.points[i].time_from_start *= speed_factor

            ### execute
            #sss.wait_for_input()
            sss.move("arm_" + self.side, "pre_grasp")
            #sss.wait_for_input()
            rospy.loginfo("approach")
            self.mgc.execute(traj_approach)
            sss.move("gripper_" + self.side, "open")
            #sss.wait_for_input()
            rospy.loginfo("grasp")
            self.mgc.execute(traj_grasp)
            #sss.wait_for_input()
            sss.move("gripper_" + self.side, "close")
            rospy.loginfo("lift")
            self.mgc.execute(traj_lift)
            #sss.wait_for_input()
            rospy.sleep(1)
            handle_arm = sss.move("arm_" + self.side, "pre_grasp")
            if handle_arm.get_error_code():
                sss.say(["script server error"])
            #sss.wait_for_input()
        return True




class MoveToTable(smach.State):
    def __init__(self, side, mode = "linear"):
        smach.State.__init__(self, 
            outcomes=['succeeded','failed'])

        # initialize tf listener
        self.listener = tf.TransformListener()
        
        self.side = side
        self.mode = mode
        
        rospy.sleep(1)
        
    def execute(self, userdata):
        #rospy.loginfo("Grasping rose...")
        #sss.wait_for_input()

        sss.say(["move to table"])

        # rough positioning
        sss.move("base","table_" + self.side, mode=self.mode)

        # fine positioning
        table_pose_offset = PoseStamped()
        table_pose_offset.header.frame_id = "table_reference"
        table_pose_offset.header.stamp = rospy.Time(0)
        table_pose_offset.pose.position.x = -0.8
        if self.side == "left":
            table_pose_offset.pose.position.y = 1.0
        elif self.side == "right":
            table_pose_offset.pose.position.y = -1.0
        else:
            rospy.logerr("invalid side: %s", self.side)
            sys.exit()
        table_pose_offset.pose.orientation.w = 1
        try:
            table_pose = self.listener.transformPose("map", table_pose_offset)
        except Exception, e:
            rospy.logerr("could not transform pose. Exception: %s", str(e))
            return "failed"

        sss.move("base", [float(table_pose.pose.position.x), float(table_pose.pose.position.y), float(tf.transformations.euler_from_quaternion([table_pose.pose.orientation.x, table_pose.pose.orientation.y, table_pose.pose.orientation.z,table_pose.pose.orientation.w])[2])], mode=self.mode)

        return "succeeded"



class Handover(smach.State):
    def __init__(self, side, mode = "linear"):
        smach.State.__init__(self, 
            outcomes=['succeeded','failed'])

        self.side = side
        self.mode = mode
        
    def execute(self, userdata):
        sss.move("base","middle", mode=self.mode)
        sss.say(["enjoy the rose"])
        sss.move("arm_" + self.side ,"side")
        #sss.wait_for_input()
        return "succeeded"



## -- State Machine 

class Roses(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,
            outcomes=['finished','failed'])
        
        self.userdata.reset_rotation = True
        
        with self:

            smach.StateMachine.add('INIT',Init(),
                transitions={'succeeded':'CHECK_FOR_STOP',
                             'failed':'failed'})

            smach.StateMachine.add('CHECK_FOR_STOP',CheckForStop(),
                transitions={'stop':'FINALIZE',
                             'continue':'SELECT_NEXT_ROSE'})

            # select next rose and table (outcome = table side)
            smach.StateMachine.add('SELECT_NEXT_ROSE',SelectCurrentRose(),
                transitions={'left':'MOVE_TO_TABLE_LEFT',
                    'right':'MOVE_TO_TABLE_RIGHT',
                    'empty_left':'CHECK_FOR_STOP',
                    'empty_right':'CHECK_FOR_STOP',
                    'failed':'finished'})

            # left table
            smach.StateMachine.add('MOVE_TO_TABLE_LEFT',MoveToTable(side="left"),
                transitions={'succeeded':'SELECT_ARM',
                    'failed':'failed'})

            # right table
            smach.StateMachine.add('MOVE_TO_TABLE_RIGHT',MoveToTable(side="right"),
                transitions={'succeeded':'SELECT_ARM',
                    'failed':'failed'})

            # select arm
            smach.StateMachine.add('SELECT_ARM',SelectArm(),
                transitions={'left':'ROTATE_ROSE_LEFT',
                    'right':'ROTATE_ROSE_RIGHT',
                    'failed':'CHECK_FOR_STOP'})

            # left arm
            smach.StateMachine.add('ROTATE_ROSE_LEFT',RotateRose("left"),
                transitions={'succeeded':'GRASP_ROSE_LEFT',
                    'failed':'CHECK_FOR_STOP'})

            smach.StateMachine.add('GRASP_ROSE_LEFT',GraspRose("left"),
                transitions={'succeeded':'HANDOVER_LEFT',
                    'failed':'ROTATE_ROSE_LEFT'})

            smach.StateMachine.add('HANDOVER_LEFT', Handover("left"),
                transitions={'succeeded':'CHECK_FOR_STOP',
                    'failed':'HANDOVER_LEFT'})

            # right arm
            smach.StateMachine.add('ROTATE_ROSE_RIGHT',RotateRose("right"),
                transitions={'succeeded':'GRASP_ROSE_RIGHT',
                    'failed':'CHECK_FOR_STOP'})

            smach.StateMachine.add('GRASP_ROSE_RIGHT',GraspRose("right"),
                transitions={'succeeded':'HANDOVER_RIGHT',
                    'failed':'ROTATE_ROSE_RIGHT'})

            smach.StateMachine.add('HANDOVER_RIGHT', Handover("right"),
                transitions={'succeeded':'CHECK_FOR_STOP',
                    'failed':'HANDOVER_RIGHT'})

            smach.StateMachine.add('FINALIZE',Finalize(),
                transitions={'succeeded':'finished',
                             'failed':'failed'})



class SM(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,outcomes=['ended'])
        with self:
            smach.StateMachine.add('STATE',Roses(),
                transitions={'finished':'ended',
                    'failed':'ended'})

if __name__=='__main__':
    rospy.init_node('roses')
    sm = SM()
    sis = smach_ros.IntrospectionServer('sm', sm, 'SM')
    sis.start()
    outcome = sm.execute()
    rospy.spin()
    sis.stop()
