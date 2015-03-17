#!/usr/bin/python
import roslib
roslib.load_manifest('hmi_scenario_states')
import rospy
import smach
import smach_ros

import random

from simple_script_server import *
sss = simple_script_server()

##
#Missing in simulation:
#Movement of the base: sss.move_base_rel("base", [0.1[m], 0.1[m], 0.1[rad]], True)

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

## -- main script
class CobIntroduction(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['succeeded','failed'])
			
    def execute(self, userdata):
        
        # :: Menue to select scenes
        while True:
            rospy.loginfo("------ Menu ------")
            rospy.loginfo("0 = Introduction & lights, mimics")
            rospy.loginfo("1 = Modules: base")
            rospy.loginfo("2 = Modules: torso")
            rospy.loginfo("3 = Modules: arms")
            rospy.loginfo("4 = Modules: head")
            usr_input = raw_input("Please type a number(int) for the scene to begin with:")
            break
        n = int(usr_input)
		
        if n == 0:
            # :: 0.Introduction & lights
            rospy.loginfo("Beginning introduction of Care-O-Bot 4-2 for HMI 2015")
            sss.move("arm_right","side", False)
            sss.move("arm_left","side")

            rospy.loginfo("Hello and Welcome")
            handle_wave = sss.move("arm_right", "wave_hmi", False)
            bool_turn = True
            while False: #"bool_turn" once movement works
                sss.move_base_rel("base", [0, 0, 0.3])
                sss.move_base_rel("base",[0, 0, -0.6])
                if not handle_wave.get_state("ACTIVE") == 1:
                    bool_turn = False
                sss.move_base_rel("base", [0, 0, 0.3], False)
            sss.say(["Hello and Welcome to my Presentation, my name is Care-O-Bot. Please dont be afraid, i am a peacefull beeing"])
            handle_wave.wait()
            sss.move("arm_right","point2chest", False)
            sss.say(["I am a mobile service robot build by Fraunhofer I. P. A., in Stuttgart and I am designed as a household assistant. My job is to help for example elderly people to stay longer at home, so that they do not have to go to a care facility."])
            sss.move("arm_right","side")
        
            # Explain lights & display
            rospy.loginfo("Showing lights & mimis")
            sss.say(["For interaction with you and expressing my mood iam able to change my colored lights and use my head-integrated display"],False)
            
            # Colors & Mimics / Not done yet
            #(...)

            sss.say(["I will show you some of my capabilities in a second."])
            rospy.loginfo("Count capabilities")

            sss.move("arm_right", "count1", False)
            sss.say(["First of all, I can interact with you, showing you my mood and intention with lights and my head-display"])
            sss.move("arm_right","count1a")
            sss.move("arm_right","count2", False)
            sss.say(["I can assist you at your home, for example, cleaning and cooking or do services in restaurants or hotels."])
            sss.move("arm_right","count2a")
            sss.move("arm_right","count3", False)
            sss.say(["or i could work in a manufacturing enviroment shelf-picking and commissioning"])
            sss.move("arm_right","count3a")
            sss.move("arm_right","count4",False)
            sss.say(["Finally , one of my technical highlights, is the modularity. Let me explain my different modules."])        
            			
            n = n+1

        if n == 1: 
            # :: 1.Explaing modules(base)
            rospy.loginfo("Explaining modules(base)")       
            sss.say(["I consist of 4 elementary parts, i'll start with my base."], False)
            sss.move("arm_right","point2base")
            sss.say(["I can move forward and backward"], False)
            sss.move_base_rel("base",[-0.1, 0, 0])
            sss.move_base_rel("base",[0.1, 0, 0])
            sss.say(["or sideways"])
            sss.move_base_rel("base",[0, 0.1, 0])
            sss.move_base_rel("base",[0, -0.1, 0])
            sss.move_base_rel("base",[0, -0.1, 0], False)
            sss.say(["and back."], False)
            sss.move_base_rel("base",[0, 0.1, 0])
            sss.say(["Iam also capable to turn on the spot like this"])
            sss.move_base_rel("base",[0, 0, 0.5])
            sss.move_base_rel("base",[0, 0, 0.5])
            sss.move_base_rel("base",[0, 0, -0.5])
            sss.move_base_rel("base",[0, 0, -0.5])
            sss.move_base_rel("base",[0, 0, -0.5])
            sss.move_base_rel("base",[0, 0, 0.5])
            sss.say(["And, of course, i can combine the movements"])

            n = n+1
        
        if n == 2:
            # :: 2.Explain modules (torso)
            rospy.loginfo("Explaining modules(torso)")
            sss.move("arm_right","point2chest")
            handle_drawtorso = sss.move("arm_right","draw_torso", False)
            sss.say(["The next module is my torso, you have already seen the lights i can change and use to interact with you, ... "])
            handle_drawtorso.wait()

            n = n+1
        
        if n == 3:
			# :: 3.Explain modules (arms)
            rospy.loginfo("Explaining modules(arms)")
            sss.move("arm_right",[[1.5, 0, 0, 0, 0, 0, 0]],False)
            sss.move("arm_left",[[-1.5, 0, 0, 0, 0, 0, 0]])
            sss.say(["as you can see, i have two arms, which i can use indepently"], False)
            r_handle = sss.move("arm_right",[[1.5, 0.3, 0, 0, 0, 0, 0]], False)
            sss.move("arm_left",[[-1.5, 0.3, 0, 0, 0, 0, 0]])           
            r_handle.wait()
            sss.move("arm_right",[[1.5, -0.3, 0, 0, 0, 0, 0]], False)
            sss.move("arm_left",[[-1.5, -0.3, 0, 0, 0, 0, 0]])
            
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
        
        if n == 4:
            # :: 4.Explain modules(head)
            rospy.loginfo("Explaining modules(head)")
            

        ## :: Final
        rospy.loginfo("Final/Exit scene reached")
        sss.say(["Thank you for your intereset"])
        sss.move_base_rel("base",[0, 0, 0.8])
        sss.say(["Thank you"])
        sss.move_base_rel("base",[0, 0, -1.6], False)
        ss.say(["Thank you for your attention"])

        #Menu to select finishing action
        rospy.loginfo("------ Menu for exit scenes ------")
        rospy.loginfo("!!!DEPENT FROM ENVIROMENT - BE CAREFUL!!!")
        rospy.loginfo("0 = arms folded")
        rospy.loginfo("1 = get cookies(HMI-2015)")
        
        user_input=raw_input("Please select how to finish the presentation")
        i = int(user_input)

        if i == 0:
            sss.move("arm_left","folded", False)
            sss.move("arm_right","folded")

        if i == 1:
            ## Find cookies, bring them to the viewers

        return 'succeeded'

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
