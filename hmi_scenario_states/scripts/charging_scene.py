#!/usr/bin/python
import roslib
roslib.load_manifest('hmi_scenario_states')
import rospy
import smach
import smach_ros

import random
import time
import atexit

from simple_script_server import *
sss = simple_script_server()

##
#Missing:
#Movement of the base: sss.move_base_rel("base", [0.1[m], 0.1[m], 1[rad]], True)

## -- Initiation
class CobIntroductionInit(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['succeeded','failed'])
			
    def execute(self, userdata):
        handle_base = sss.init("base")
        handle_arm_left = sss.init("arm_left")
        handle_arm_right = sss.init("arm_right")
        handle_sensorring = sss.init("sensorring")

        if handle_base.get_error_code() != 0:
            return "failed"
        if handle_arm_left.get_error_code() != 0:
            return "failed"
        if handle_arm_right.get_error_code() != 0:
            return "failed"
        if handle_sensorring.get_error_code() != 0:
            return "failed"

        return "succeeded"
## -- Recover
class CobIntroductionRecover(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['succeeded','failed'])
			
    def execute(self, userdata):
        handle_base = sss.recover("base")
        handle_arm_left = sss.recover("arm_left")
        handle_arm_right = sss.recover("arm_right")
        handle_sensorring = sss.recover("sensorring")

        if handle_base.get_error_code() != 0:
            return "failed"
        if handle_arm_left.get_error_code() != 0:
            return "failed"
        if handle_arm_right.get_error_code() != 0:
            return "failed"
        if handle_sensorring.get_error_code() != 0:
            return "failed"

        return "succeeded"

## -- Prepare
class CobIntroductionPrepare(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['succeeded','failed'])
			
    def execute(self, userdata):
        handle_arm_left = sss.move("arm_left", "folded",False)
        handle_arm_right = sss.move("arm_right", "folded",False)
        handle_sensorring = sss.move("sensorring", "front",False)

        handle_arm_left.wait()
        handle_arm_right.wait()
        handle_sensorring.wait()

        if handle_arm_left.get_error_code() != 0:
            return "failed"
        if handle_arm_right.get_error_code() != 0:
            return "failed"
        if handle_sensorring.get_error_code() != 0:
            return "failed"

        return "succeeded"

## -- charhing_scene main script
class CobIntroduction(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['succeeded','failed'])
			
    def execute(self, userdata):
        
        # :: Menue to select charging scenes
        while True:
            rospy.loginfo("0 = Do nothing(arms folded)")
            rospy.loginfo("1 = Do nothing(arms outrstretched)")
            rospy.loginfo("2 = Thinking (needs place to move)")
            rospy.loginfo("3 = Gestures, randomized timing (needs place for arms)")
            rospy.loginfo("4 = Wave right arm")
            while True:
                try: 
                    usr_input = raw_input("Please type a number(int) for the scene to begin with:")
                    n = int(usr_input)
                    break
                except ValueError:
                    rospy.loginfo("You didn't type a number, please try again.")
            break
        n = int(usr_input)
		
        #Do nothing(arms folded)
        if n == 0:
            sss.move("arm_right","folded",False)
            sss.move("arm_left","folded")
            ## To do: Mimics
 
        #Do nothing(arms outstretched)
        if n == 1:
            sss.move("arm_right","home",False)
            sss.move("arm_left","home")
            ## To do: Mimics

        #Walk around thinking
        if n == 2:
            sss.move("arm_right","folded",False)
            sss.move("arm_left",[[0.4, 1.5, -1, 2.2, 0 , 0.4, 0]])
            ## To do: walk left, turn, walrk right

        #Gestures with randomized timing
        if n == 3:

            random.seed()
        
            while True:
                n=random.random()
                time.sleep(n*60)

                #Thinking
                rospy.loginfo("Cob: Thinking")
                sss.move("arm_right","folded")
                sss.move("arm_left",[[0.4, 1.5, -1, 2.2, 0 , 0.4, 0]])
                
                n=random.random()
                time.sleep(n*60)
                sss.move("arm_right", "folded")
                sss.move("arm_left", "folded")
                n=random.random()
                time.sleep(n*60)

                #Wave left
                ("Cob: Waving left")
                sss.move("arm_left", "wave_hmi")
                sss.move("arm_left", "wave_hmi")
                sss.move("arm_left", "folded")

                n=random.random()
                time.sleep(n*60)

                #Wave right
                ("Cob: waving right")
                sss.move("arm_right", "wave_hmi")
                sss.move("arm_right", "wave_hmi")
                sss.move("arm_right", "folded")
        
        #Wave right arm
        if n == 4:
            loop_exit = True
            while True:
                sss.move("arm_right", "wave_hmi")

        return "succeeded"


## -- State Machine 

class Explore(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,
            outcomes=['finished','failed'])
        with self:

            smach.StateMachine.add('COB_INTRODUCTION_PREPARE',CobIntroductionPrepare(),
                transitions={'succeeded':'COB_INTRODUCTION',
                    'failed':'failed'})

            smach.StateMachine.add('COB_INTRODUCTION',CobIntroduction(),
                transitions={'succeeded':'finished',
                    'failed':'failed'})

















class SM(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,outcomes=['ended'])
        with self:
            smach.StateMachine.add('STATE',Explore(),
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
