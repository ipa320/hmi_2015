#!/usr/bin/python
import roslib
roslib.load_manifest('hmi_scenario_states')
import rospy
import smach
import smach_ros

import random

from simple_script_server import *
sss = simple_script_server()

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

class CobIntroductionPrepare(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['succeeded','failed'])
			
    def execute(self, userdata):
        handle_arm_left = sss.move("arm_left", "home",False)
        handle_arm_right = sss.move("arm_right", "home",False)
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

class CobIntroduction(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['succeeded','failed'])
			
    def execute(self, userdata):
    
        while True:
            rospy.loginfo("0 = Introduction")
            rospy.loginfo("1 = Modules: base")
            rospy.loginfo("2 = Modules: torso")
            rospy.loginfo("3 = Modules: arms")
            usr_input = raw_input("Please type a number(int) for the scene to begin with:")
            break
        n = int(usr_input)
		
        if n == 0:
            #::::::Introduction
            rospy.loginfo("Beginning introduction of Care-O-Bot 4-2 for HMI 2015")
            sss.move("arm_right","side", False)
            sss.move("arm_left","side")

            rospy.loginfo("Scene 1 : Hello and Welcome")
            handle_wave = sss.move("arm_right", "wave_hmi", False)
            sss.say(["Hello and Welcome to my Presentation, my name is Care-O-Bot. Please dont be afraid, i am a peacefull beeing"])
            handle_wave.wait()
            sss.move("arm_right","point2chest")
            sss.say(["I am a mobile service robot build by Fraunhofer I. P. A., in Stuttgart and I am designed as a household assistant. My job is to help for example elderly people to stay longer at home, so that they do not have to go to a care facility."])
        
            sss.say(["I will show you some of my capabilities in a second."])
            rospy.loginfo("Scene 2 : Count capabilities")

            handle_count = sss.move("arm_right", "count", False)
            sss.say("I can show you my mood and intention with lights and my head-display")
            sss.say("I am also able to recognize different types of objekts wich i can manipulate and interact with")
            handle_count.wait()
			
            n = n+1

        if n == 1:
            #::::::Explain modules (base)
            rospy.loginfo("Explaining modules(base)")
            sss.say(["For interaction with you and expressing my mood iam able to change my colored lights and use my head-integrated display"])
            # Colors & Mimics / Not done yet

            #(...)
        
            sss.say(["I consist of 4 elementary parts, i'll start with my base."])
            # Move Base
            sss.say(["I can move forwar and backward, or to the side"])
            # Turn Base
            sss.say(["Iam also capable to turn on the spot like this"])
            # Turn and Move Base
            sss.say(["And, of course, i can combine the movements"])

            n = n+1
        
        if n == 2:
            #::::::Explain modules (torso)
            rospy.loginfo("Explaining modules(torso)")
            sss.move("arm_right","point2chest")
            handle_drawtorso = sss.move("arm_right","draw_torso", False)
            sss.say(["The next module is my torso, you have already seen the lights i can change and use to interact with you, ... "])
            handle_drawtorso.wait()

            n = n+1
        
        if n == 3:
			#::::::Explain modules (arms)
            rospy.loginfo("Explaining modules(arms)")
            sss.move("arm_right", "home", False)
            sss.move("arm_left", "home", False)
            sss.say(["as you can see, i have two arms, which i can use indepently"])
            #sss.move("arm_right", "rotate", False)
            #sss.move("arm_left", "inv_rotate", False)
            sss.say(["Both arms consist of 7 independet joints, allowing me to perform complex movements"], False)

            rospy.loginfo("Beginning arena-like wave")
            sss.move("arm_right",[[1.5, 0, 0, 0, 0, 0, 0]],False)
            sss.move("arm_left",[[-1.5, 0, 0, 0, 0, 0, 0]])
            sss.move("arm_right",[[1.5, 0, 0, 0, 0, 0.5, 0]])
            sss.move("arm_right",[[1.5, -0.3, 0, 1, 0, -0.2, 0]])

            sss.move("arm_left",[[-1.5, 0.3, 0, -1, 0, 0.2, 0]], False)
            sss.move("arm_right",[[1.5, 0.3, 0, -1, 0, 0.2, 0]])

            sss.move("arm_left",[[-1.5, -0.3, 0, 1, 0, -0.2, 0]], False)
            sss.move("arm_right",[[1.5, -0.3, 0, 1, 0, -0.2, 0]])

            sss.move("arm_right","side", False)
            handle_lastmv = sss.move("arm_left","folded")
            handle_lastmv.wait()
			
            n = n+1


        

        return 'succeeded'

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
