#!/usr/bin/python
import rospy
import rospkg
import smach
import smach_ros
import tf
import sys
import copy

import random

from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import RobotState, AttachedCollisionObject, CollisionObject
from shape_msgs.msg import SolidPrimitive
import simple_moveit_interface as smi

from simple_script_server import *
sss = simple_script_server()

def move_gripper(component_name, pos):
    error_code = -1
    counter = 0
    while not rospy.is_shutdown() and error_code != 0:
        print "trying to move", component_name, "to", pos, "retries: ", counter
        #rospy.sleep(0.1)
        handle = sss.move(component_name, pos)
        handle.wait()
        error_code = handle.get_error_code()
        if counter > 5:
        		sss.set_light("light_torso","blue")
        if counter > 100:
            sss.set_light("light_torso","red")
            sss.say([component_name + " error, please help me"])
            rospy.logerr(component_name + "does not work any more. retries: " + str(counter) + ". Please reset USB connection and press <ENTER>.")
            sss.wait_for_input()
            sss.set_light("light_torso","cyan")
            return False
        counter += 1
    sss.set_light("light_torso","cyan")
    return True

class Init(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','failed'],
            output_keys=['reset_roses'])

    def execute(self, userdata):
        sss.set_light("light_base","green")
        sss.set_light("light_torso","green")
        rospy.sleep(0.5)
        sss.set_light("light_base","cyan")
        sss.set_light("light_torso","cyan")
        rospy.sleep(0.5)
        sss.set_light("light_base","green")
        sss.set_light("light_torso","green")
        rospy.sleep(0.5)
        sss.set_light("light_base","cyan")
        sss.set_light("light_torso","cyan")
        rospy.sleep(0.5)
        
        #handle_gripper_left = sss.move("gripper_left","home")
        #handle_gripper_right = sss.move("gripper_right","home")
        #handle_gripper_left.wait()
        #handle_gripper_right.wait()
        move_gripper("gripper_left", "home")
        move_gripper("gripper_right", "home")

        handle_arm_left = sss.move("arm_left","side",False)
        handle_arm_right = sss.move("arm_right","side",False)
        handle_arm_left.wait()
        handle_arm_right.wait()

        sss.move("base","middle",mode="linear")
        
        sss.set_light("light_base","cyan")
        sss.set_light("light_torso","cyan")
        
        userdata.reset_roses = True
        return "succeeded"

class CheckForStop(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['stop','continue'])
        self.stop_requested = False
        rospy.Service('/hmi/stop_roses', Trigger, self.stop_cb)
        rospy.Service('/hmi/continue_roses', Trigger, self.continue_cb)
            
    def execute(self, userdata):
        if self.stop_requested:
            self.stop_requested = False
            return "stop"
        else:
            return "continue"

    def stop_cb(self, req):
        print "stop roses"
        self.stop_requested = True
        res = TriggerResponse()
        res.success = True
        return res

    def continue_cb(self, req):
        print "continue roses"
        self.stop_requested = False
        res = TriggerResponse()
        res.success = True
        return res

class Finalize(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','failed'])

    def execute(self, userdata):

        #handle_gripper_left = sss.move("gripper_left","home")
        #handle_gripper_right = sss.move("gripper_right","home")
        #handle_gripper_left.wait()
        #handle_gripper_right.wait()
        move_gripper("gripper_left", "home")
        move_gripper("gripper_right", "home")

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
            outcomes=['left','right','failed','empty'],
            input_keys=['reset_roses'],
            output_keys=['reset_rotation','reset_roses'])

        self.current_rose = ""
        rospy.Timer(rospy.Duration(0.1), self.broadcast_tf)
        self.br = tf.TransformBroadcaster()
        (self.roses_left, self.roses_right) = self.fill_roses() # contains names of roses
            
    def execute(self, userdata):
        if userdata.reset_roses:
            (self.roses_left, self.roses_right) = self.fill_roses() # contains names of roses
            userdata.reset_roses = False
        print "roses_left =", self.roses_left
        print "roses_right =", self.roses_right
        if len(self.roses_right) > 0:
            self.current_rose = self.roses_right[0]

            # check if this rose is in first row
            xes = []
            for rose in self.roses_right:
                xes.append(rose.split("_")[2].split("-")[0])
            current_x = self.current_rose.split("_")[2].split("-")[0]
            if current_x == min(xes):
                rospy.loginfo("selected %s", self.current_rose)
                #sss.say(["selected " + self.current_rose], False)
                rospy.sleep(0.5) #FIXME: wait for TF to update
                userdata.reset_rotation = True
                # remove current rose from list
                self.current_rose = self.roses_right.pop(0)#FIXME only pop if grasping is successfull
                return "right"
            else:
                print "no more graspable roses right"
                self.roses_right = []

        if len(self.roses_left) > 0:
            self.current_rose = self.roses_left[0]

            # check if this rose is in first row
            xes = []
            for rose in self.roses_left:
                xes.append(rose.split("_")[2].split("-")[0])
            current_x = self.current_rose.split("_")[2].split("-")[0]
            if current_x == min(xes):
                rospy.loginfo("selected %s", self.current_rose)
                #sss.say(["selected " + self.current_rose], False)
                rospy.sleep(0.5) #FIXME: wait for TF to update
                userdata.reset_rotation = True
                # remove current rose from list
                self.current_rose = self.roses_left.pop(0)#FIXME only pop if grasping is successfull
                return "left"
            else:
                print "no more graspable roses left"
                self.roses_left = []

        rospy.logwarn("No more roses, please fill up")
        #sss.say(["I delivered all roses from the table. Please provide me with more roses."],False)
        (self.roses_left, self.roses_right) = self.fill_roses() # contains names of roses
        return "empty"

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
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['succeeded','failed'],
            input_keys=['reset_rotation', 'active_arm'],
            output_keys=['reset_rotation'])

        rospy.Timer(rospy.Duration(0.1), self.broadcast_tf)
        self.br = tf.TransformBroadcaster()
        self.angle_offset_roll = 0
        self.extreme_angle_yaw_1 = 30.0/180.0*math.pi#-30.0/180.0*math.pi
        self.extreme_angle_yaw_2 = 60.0/180.0*math.pi
        self.angle_step_yaw = 15.0/180.0*math.pi
        self.angle_offset_yaw = self.extreme_angle_yaw_1

        rospy.sleep(1)
            
    def execute(self, userdata):
        if userdata.active_arm == "left":
            self.angle_offset_roll = math.pi
            angle_step_yaw = -self.angle_step_yaw
            if userdata.reset_rotation:
                self.angle_offset_yaw = -self.extreme_angle_yaw_1
                userdata.reset_rotation = False
            else:
                if math.fabs(self.angle_offset_yaw) < math.fabs(self.extreme_angle_yaw_1) or math.fabs(self.angle_offset_yaw) > math.fabs(self.extreme_angle_yaw_2):
                    # reset angle_offset_yaw
                    self.angle_offset_yaw = -self.extreme_angle_yaw_1
                    #sss.say(["Unfortunately I can not reach this rose. I will try the next one."], False)
                    return "failed"
                else:
                    self.angle_offset_yaw += angle_step_yaw
        elif userdata.active_arm == "right":
            self.angle_offset_roll = 0
            angle_step_yaw = self.angle_step_yaw
            if userdata.reset_rotation:
                self.angle_offset_yaw = self.extreme_angle_yaw_1
                userdata.reset_rotation = False
            else:
                if math.fabs(self.angle_offset_yaw) < math.fabs(self.extreme_angle_yaw_1) or math.fabs(self.angle_offset_yaw) > math.fabs(self.extreme_angle_yaw_2):
                    # reset angle_offset_yaw
                    self.angle_offset_yaw = self.extreme_angle_yaw_1
                    #sss.say(["Unfortunately I can not reach this rose. I will try the next one."], False)
                    return "failed"
                else:
                    self.angle_offset_yaw += angle_step_yaw
        else:
            rospy.logerr("invalid active_arm: %s", userdata.active_arm)
            sys.exit()
        
        #sss.say(["angle offset yaw is " + str(int(round(self.angle_offset_yaw/math.pi*180)))])
        print "angle offset yaw is " + str(int(round(self.angle_offset_yaw/math.pi*180)))
        rospy.sleep(1) # FIXME: wait for TF to update
        
        return "succeeded"

    def broadcast_tf(self, event):
        self.br.sendTransform(
                (0, 0, 0),
                 tf.transformations.quaternion_from_euler(self.angle_offset_roll, 0, self.angle_offset_yaw),
                 event.current_real,
                 "current_rose",
                 "current_rose_fixed")

class SelectArm(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['succeeded', 'failed'],
            output_keys=['reset_rotation','active_arm'])

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
            userdata.active_arm = "left"
        else:
            userdata.active_arm = "right"
        
        userdata.reset_rotation = True
        return 'succeeded'

class GraspRose(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['succeeded','failed'],
            input_keys=['active_arm'],
            output_keys=['reset_rotation'])

        # initialize tf listener
        self.listener = tf.TransformListener()
        
        ### Create a handle for the Move Group Commander
        self.mgc_left = MoveGroupCommander("arm_left")
        self.mgc_right = MoveGroupCommander("arm_right")
        
        ### Create a handle for the Planning Scene Interface
        self.psi = PlanningSceneInterface()
        
        self.eef_step = 0.01
        self.jump_threshold = 2
        
        rospy.sleep(1)
        
    def execute(self, userdata):
        #rospy.loginfo("Grasping rose...")
        #sss.wait_for_input()

        if not self.plan_and_execute(userdata):
            userdata.reset_rotation = False
            return "failed"
        
        return "succeeded"
           


    def plan_and_execute(self, userdata):
        # add table
        ps = PoseStamped()
        ps.header.frame_id = "table_top"
        ps.pose.position.x = -0.05
        ps.pose.position.z = 0.05
        ps.pose.orientation.w = 1
        filename = rospkg.RosPack().get_path("hmi_manipulation") + "/files/hmi_table.stl"
        self.psi.add_mesh("table", ps, filename)

        ### Set next (virtual) start state
        start_state = RobotState()
        (pre_grasp_config, error_code) = sss.compose_trajectory("arm_" + userdata.active_arm,"pre_grasp")
        if error_code != 0:
            rospy.logerr("unable to parse pre_grasp configuration")
            return False
        start_state.joint_state.name = pre_grasp_config.joint_names
        start_state.joint_state.position = pre_grasp_config.points[0].positions
        start_state.is_diff = True
        if userdata.active_arm == "left":
            self.mgc_left.set_start_state(start_state)
        elif userdata.active_arm == "right":
            self.mgc_right.set_start_state(start_state)
        else:
            rospy.logerr("invalid arm_active")
            return False

        ### Plan Approach
        approach_pose_offset = PoseStamped()
        approach_pose_offset.header.frame_id = "current_rose"
        approach_pose_offset.header.stamp = rospy.Time(0)
        approach_pose_offset.pose.position.x = -0.12
        approach_pose_offset.pose.orientation.w = 1
        try:
            approach_pose = self.listener.transformPose("odom_combined", approach_pose_offset)
        except Exception, e:
            rospy.logerr("could not transform pose. Exception: %s", str(e))
            return False
        
        if userdata.active_arm == "left":
            (traj_approach,frac_approach) = self.mgc_left.compute_cartesian_path([approach_pose.pose], self.eef_step, self.jump_threshold, True)
        elif userdata.active_arm == "right":
            (traj_approach,frac_approach) = self.mgc_right.compute_cartesian_path([approach_pose.pose], self.eef_step, self.jump_threshold, True)
        else:
            rospy.logerr("invalid arm_active")
            return False
        
        traj_approach = self.smooth_cartesian_path(traj_approach)
        
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
        if userdata.active_arm == "left":
            self.mgc_left.set_start_state(start_state)
        elif userdata.active_arm == "right":
            self.mgc_right.set_start_state(start_state)
        else:
            rospy.logerr("invalid arm_active")
            return False

        ### Plan Grasp
        grasp_pose_offset = PoseStamped()
        grasp_pose_offset.header.frame_id = "current_rose"
        grasp_pose_offset.header.stamp = rospy.Time(0)
        grasp_pose_offset.pose.orientation.w = 1
        grasp_pose = self.listener.transformPose("odom_combined", grasp_pose_offset)
        if userdata.active_arm == "left":
            (traj_grasp,frac_grasp) = self.mgc_left.compute_cartesian_path([grasp_pose.pose], self.eef_step, self.jump_threshold, True)
        elif userdata.active_arm == "right":
            (traj_grasp,frac_grasp) = self.mgc_right.compute_cartesian_path([grasp_pose.pose], self.eef_step, self.jump_threshold, True)
        else:
            rospy.logerr("invalid arm_active")
            return False
        
        traj_grasp = self.smooth_cartesian_path(traj_grasp)
        
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
        if userdata.active_arm == "left":
            self.mgc_left.set_start_state(start_state)
        elif userdata.active_arm == "right":
            self.mgc_right.set_start_state(start_state)
        else:
            rospy.logerr("invalid arm_active")
            return False

        ### Plan Lift
        lift_pose_offset = PoseStamped()
        lift_pose_offset.header.frame_id = "current_rose"
        lift_pose_offset.header.stamp = rospy.Time(0)
        if userdata.active_arm == "left":
            lift_pose_offset.pose.position.z = -0.2#-0.3#-0.12
        elif userdata.active_arm == "right":
            lift_pose_offset.pose.position.z = 0.2#0.3#0.12
        else:
            rospy.logerr("invalid active_arm: %s", userdata.active_arm)
            sys.exit()
        lift_pose_offset.pose.orientation.w = 1
        lift_pose = self.listener.transformPose("odom_combined", lift_pose_offset)

        if userdata.active_arm == "left":
            (traj_lift,frac_lift) = self.mgc_left.compute_cartesian_path([lift_pose.pose], self.eef_step, self.jump_threshold, True)
        elif userdata.active_arm == "right":
            (traj_lift,frac_lift) = self.mgc_right.compute_cartesian_path([lift_pose.pose], self.eef_step, self.jump_threshold, True)
        else:
            rospy.logerr("invalid arm_active")
            return False
        
        traj_lift = self.smooth_cartesian_path(traj_lift)
        
        if not (frac_lift == 1.0):
            rospy.logerr("Unable to plan lift trajectory")
            #sss.say(["no lift trajectory: skipping rose"], False)
            return False

        """
        ### Set next (virtual) start state
        traj_lift_endpoint = traj_lift.joint_trajectory.points[-1]
        start_state = RobotState()
        start_state.joint_state.name = traj_lift.joint_trajectory.joint_names
        start_state.joint_state.position = traj_lift_endpoint.positions
        
        rose_primitive = SolidPrimitive()
        rose_primitive.type = 3 #CYLINDER
        rose_height = 0.4
        rose_radius = 0.05
        rose_primitive.dimensions.append(rose_height)
        rose_primitive.dimensions.append(rose_radius)
        
        rose_pose = Pose()
        rose_pose.orientation.w = 1.0
        
        rose_collision = CollisionObject()
        rose_collision.header.frame_id = "gripper_"+userdata.active_arm+"_grasp_link"
        rose_collision.id = "current_rose"
        rose_collision.primitives.append(rose_primitive)
        rose_collision.primitive_poses.append(rose_pose)
        rose_collision.operation = 0 #ADD
        
        rose_attached = AttachedCollisionObject()
        rose_attached.link_name = "gripper_"+userdata.active_arm+"_grasp_link"
        rose_attached.object = rose_collision
        rose_attached.touch_links = ["gripper_"+userdata.active_arm+"_base_link", "gripper_"+userdata.active_arm+"_camera_link", "gripper_"+userdata.active_arm+"_finger_1_link", "gripper_"+userdata.active_arm+"_finger_2_link", "gripper_"+userdata.active_arm+"_grasp_link", "gripper_"+userdata.active_arm+"_palm_link"]
        
        start_state.attached_collision_objects.append(rose_attached)
        
        start_state.is_diff = True
        if userdata.active_arm == "left":
            self.mgc_left.set_start_state(start_state)
        elif userdata.active_arm == "right":
            self.mgc_right.set_start_state(start_state)
        else:
            rospy.logerr("invalid arm_active")
            return False

        #Plan retreat
        pre_grasp_config = smi.get_goal_from_server("arm_"+userdata.active_arm, "pre_grasp")
        #print pre_grasp_config
        
        if pre_grasp_config == None:
            rospy.logerr("GoalConfig not found on ParameterServer")
            #sss.say(["GoalConfig not found on ParameterServer"], False)
            return False
        
        if userdata.active_arm == "left":
            self.mgc_left.set_planner_id("RRTkConfigDefault")
            traj_pre_grasp = self.mgc_left.plan(pre_grasp_config)
        elif userdata.active_arm == "right":
            self.mgc_right.set_planner_id("RRTkConfigDefault")
            traj_pre_grasp = self.mgc_right.plan(pre_grasp_config)
        else:
            rospy.logerr("invalid arm_active")
            return False
        print traj_pre_grasp
        
        if traj_pre_grasp == None:
            rospy.logerr("Unable to plan pre_grasp trajectory")
            #sss.say(["no pre_grasp trajectory: skipping rose"], False)
            return False
        """
        #if not (frac_approach == 1.0 and frac_grasp == 1.0 and frac_lift == 1.0 and not traj_pre_grasp == None):
        if not (frac_approach == 1.0 and frac_grasp == 1.0 and frac_lift == 1.0):
            rospy.logerr("Unable to plan whole grasping trajectory")
            #sss.say(["skipping rose"], False)
            return False
        else:
            #sss.say(["grasping rose"], False)

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
            sss.move("arm_" + userdata.active_arm, "pre_grasp")
            #sss.wait_for_input()
            rospy.loginfo("approach")
            if userdata.active_arm == "left":
                self.mgc_left.execute(traj_approach)
                #handle_gripper = sss.move("gripper_" + userdata.active_arm, "open")
                move_gripper("gripper_" + userdata.active_arm, "open")
                #sss.wait_for_input()
                rospy.loginfo("grasp")
                self.mgc_left.execute(traj_grasp)
                #sss.wait_for_input()
                #sss.move("gripper_" + userdata.active_arm, "close")
                move_gripper("gripper_" + userdata.active_arm, "close")
                rospy.loginfo("lift")
                self.mgc_left.execute(traj_lift)
                #sss.wait_for_input()
                #self.mgc_left.execute(traj_pre_grasp)
                #rospy.sleep(1)
                sss.move("base","middle", mode="linear", blocking=False)
                #rospy.sleep(0.5) #wait for base to move away from table
                handle_arm = sss.move("arm_" + userdata.active_arm, "retreat")
            elif userdata.active_arm == "right":
                self.mgc_right.execute(traj_approach)
                #sss.move("gripper_" + userdata.active_arm, "open")
                move_gripper("gripper_" + userdata.active_arm, "open")
                #sss.wait_for_input()
                rospy.loginfo("grasp")
                self.mgc_right.execute(traj_grasp)
                #sss.wait_for_input()
                #sss.move("gripper_" + userdata.active_arm, "close")
                move_gripper("gripper_" + userdata.active_arm, "close")
                rospy.loginfo("lift")
                self.mgc_right.execute(traj_lift)
                #sss.wait_for_input()
                #self.mgc_right.execute(traj_pre_grasp)
                #rospy.sleep(1)
                sss.move("base","middle", mode="linear", blocking=False)
                #rospy.sleep(0.5) #wait for base to move away from table
                handle_arm = sss.move("arm_" + userdata.active_arm, "retreat")
            else:
                rospy.logerr("invalid arm_active")
                return False
            
            
            #if handle_arm.get_error_code():
            #    #sss.say(["script server error"])
            #    rospy.logerr("script server error")
            #    sss.set_light("light_base","red")
            #sss.wait_for_input()
        return True
        
    def smooth_cartesian_path(self, traj):
        #print traj
        time_offset = 0.2
        
        for i in range(len(traj.joint_trajectory.points)):
            traj.joint_trajectory.points[i].time_from_start += rospy.Duration(time_offset)
        
        traj.joint_trajectory.points[-1].time_from_start += rospy.Duration(time_offset)
        
        
        #print "\n\n\n"
        #print traj
        return traj


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

        #sss.say(["move to table"])

        # rough positioning
        sss.move("base","table_" + self.side, mode=self.mode)

        # fine positioning
        table_pose_offset = PoseStamped()
        table_pose_offset.header.frame_id = "table_reference"
        table_pose_offset.header.stamp = rospy.Time(0)
        table_pose_offset.pose.position.x = -0.8
        if self.side == "left":
            table_pose_offset.pose.position.y = 1.1
        elif self.side == "right":
            table_pose_offset.pose.position.y = -1.1
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
    def __init__(self, mode = "linear"):
        smach.State.__init__(self, 
            outcomes=['succeeded','failed'],
            input_keys=['active_arm'])

        # service servers
        rospy.Service('/hmi/handover_none', Trigger, self.handover_none_cb)
        rospy.Service('/hmi/handover_left', Trigger, self.handover_left_cb)
        rospy.Service('/hmi/handover_right', Trigger, self.handover_right_cb)
        rospy.Service('/hmi/handover_middle', Trigger, self.handover_middle_cb)
        rospy.Service('/hmi/handover_offer_rose', Trigger, self.offer_rose_cb)
        rospy.Service('/hmi/handover_release_rose', Trigger, self.release_rose_cb)

        self.mode = mode
        self.handover_pos = ""
        self.offer = False
        self.release = False

    def handover_none_cb(self, req):
        self.offer = True
        self.handover_pos = "NONE"
        print "selected", self.handover_pos
        return TriggerResponse()
    def handover_left_cb(self, req):
        self.handover_pos = "handover_left"
        print "selected", self.handover_pos
        return TriggerResponse()
    def handover_right_cb(self, req):
        self.handover_pos = "handover_right"
        print "selected", self.handover_pos
        return TriggerResponse()
    def handover_middle_cb(self, req):
        self.handover_pos = "handover_middle"
        print "selected", self.handover_pos
        return TriggerResponse()
    def offer_rose_cb(self, req):
        self.offer = True
        return TriggerResponse()
    def release_rose_cb(self, req):
        self.release = True
        return TriggerResponse()

    def execute(self, userdata):
        self.offer = False
        self.release = False
        self.handover_pos = ""           
        
        # move to handover pos
        sss.set_light("light_base","green")
        while not rospy.is_shutdown() and self.handover_pos == "" and not self.offer:
            rospy.loginfo("waiting for handover pos or use joystick")
            rospy.sleep(1)
        
        if self.handover_pos == "NONE":
            sss.set_light("light_base","cyan")
            sss.move("arm_" + userdata.active_arm ,"side", False)
            move_gripper("gripper_" + userdata.active_arm, "home")
            self.offer = False
            self.release = False
            self.handover_pos = ""
            return "succeeded" 
        elif self.handover_pos != "":
            sss.set_light("light_base","cyan")
            sss.move("base",self.handover_pos, mode=self.mode)
        #self.handover_pos = ""
        
        # offer
        sss.set_light("light_base","green")
        while not rospy.is_shutdown() and not self.offer:
            rospy.loginfo("waiting for offer trigger")
            rospy.sleep(1)
        self.offer = False
        
        sss.set_light("light_base","cyan")
        sss.move("arm_" + userdata.active_arm ,"handover")
        sss.say(["This rose is for you. Have a good day."])
        
        # release
        sss.set_light("light_base","green")
        while not rospy.is_shutdown() and not self.release:
            rospy.loginfo("waiting for release trigger")
            rospy.sleep(1)
        self.release = False  
        
        sss.set_light("light_base","cyan")
        #sss.move("gripper_" + userdata.active_arm ,"open")
        move_gripper("gripper_" + userdata.active_arm, "open")
        rospy.sleep(1)
        #sss.move("gripper_" + userdata.active_arm ,"home", False)
        move_gripper("gripper_" + userdata.active_arm, "home")
        sss.move("arm_" + userdata.active_arm ,"side")
        
        sss.move("base",self.handover_pos+"_post", mode=self.mode)
        self.handover_pos = ""

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
                    'empty':'CHECK_FOR_STOP',
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
                transitions={'succeeded':'ROTATE_ROSE',
                    'failed':'CHECK_FOR_STOP'})

            # grasp
            smach.StateMachine.add('ROTATE_ROSE',RotateRose(),
                transitions={'succeeded':'GRASP_ROSE',
                    'failed':'CHECK_FOR_STOP'})

            smach.StateMachine.add('GRASP_ROSE',GraspRose(),
                transitions={'succeeded':'HANDOVER',
                    'failed':'ROTATE_ROSE'})

            # handover
            smach.StateMachine.add('HANDOVER', Handover(),
                transitions={'succeeded':'CHECK_FOR_STOP',
                    'failed':'FINALIZE'})

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
