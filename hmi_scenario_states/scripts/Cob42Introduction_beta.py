#!/usr/bin/python
import roslib
roslib.load_manifest('hmi_scenario_states')
import rospy
import smach
import smach_ros
import sys

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
        #handle_sensorring = sss.init("sensorring")

        if handle_base.get_error_code() != 0:
            return "failed"
        if handle_arm_left.get_error_code() != 0:
            return "failed"
        if handle_arm_right.get_error_code() != 0:
            return "failed"
        #if handle_sensorring.get_error_code() != 0:
            #return "failed"

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
        #handle_sensorring = sss.recover("sensorring")

        if handle_base.get_error_code() != 0:
            return "failed"
        if handle_arm_left.get_error_code() != 0:
            return "failed"
        if handle_arm_right.get_error_code() != 0:
            return "failed"
        #if handle_sensorring.get_error_code() != 0:
            #return "failed"

        return "succeeded"

## -- Prepare
class CobIntroductionPrepare(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['succeeded','failed'])
			
    def execute(self, userdata):
        handle_arm_left = sss.move("arm_left", "folded",False)
        handle_arm_right = sss.move("arm_right", "folded",False)
        #handle_sensorring = sss.move("sensorring", "front",False)

        handle_arm_left.wait()
        handle_arm_right.wait()
        #handle_sensorring.wait()

        if handle_arm_left.get_error_code() != 0:
            return "failed"
        if handle_arm_right.get_error_code() != 0:
            return "failed"
        #if handle_sensorring.get_error_code() != 0:
            r#eturn "failed"

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
            rospy.loginfo("5 = Software highlights")
            while True:
                try:          
                    usr_input = raw_input("Please type a number(int) for the scene to begin with:")
                    n = int(usr_input)
                    #n=0
                    break
                except ValueError:
                    rospy.loginfo("You didn't type a number, please try again")
            break
		
        if n == 0:
            # :: 0.Introduction & lights
            sss.set_mimic("mimic","happy")
            rospy.loginfo("Beginning introduction of Care-O-bot 4-2 for HMI 2015")
            sss.move("arm_right", "side", False)
            sss.move("arm_left", "side")

            # sync
            sss.say(["Hello and welcome to my presentation, my name is Care o bot. I am a mobile service robot build by Fraunhofer I. P. A., in Stuttgart."], False)
            sss.move_base_rel("base", [0, 0, -0.78], False)
            rospy.sleep(0.5)
            sss.move("arm_right", "wave_hmi")

            # sync

            sss.say(["Dont be afraid, I am a gentleman"], False)
            sss.move_base_rel("base", [0, 0, 1.3], False)
            rospy.sleep(1)
            sss.say(["I have a wide range of services. I can assist you at home or serve food and drinks in restaurants or hotels. In hospitals and care facilities I can give support in various delivery tasks."], False) #TODO show renderings on head display while explaining application domanis
            handle_arm_left = sss.move("arm_left", "wave_hmi", False)
            
            rospy.sleep(8)

            sss.move_base_rel("base", [0, 0, -0.78])
            handle_arm_left.wait()
            
            # sync
            n = n+1

        if n == 1:
            handle_arm_right = sss.move("arm_right","point2head", False)

            # Explain lights & display
            sss.set_mimic("mimic",["laughing",0,1])
            sss.say(["I can use my eyes for interacting with humans."])
            rospy.sleep(2)
            sss.move("arm_left","point2base", False)
            sss.say(["and express my mood by changing color in my torso and base."])
            handle_arm_right.wait()

            # sync
            rospy.sleep(0.5)
            handle_say = sss.say(["If I light up in yellow that means that I am moving my base or my arms, so please pay attention"], False)
            sss.move_base_rel("base", [0, 0, -0.35], False)
            sss.set_mimic("mimic", ["busy",0,1], False)
            rospy.sleep(1)
            sss.move("arm_right","point2base", False)
            sss.set_light("light_base","yellow", False)
            sss.set_light("light_torso","yellow", False)
            handle_say.wait()
            
            sss.move_base_rel("base", [0, 0, 0.35], False)
            rospy.sleep(3)

            # sync
            handle_say = sss.say(["If you see me in red, there is an error and I need help. But dont worry, I feel good right now, this is only to show you my colors. I hope you will never see me like this"], False)
            sss.set_mimic("mimic", ["confused",0,1], False)
            sss.set_light("light_base","red", False)
            sss.set_light("light_torso","red", False)
            rospy.sleep(0.5)
            handle_say.wait()

            # sync
            handle_say = sss.say(["My normal color is this, showing you that I am ready to be at your service."], False)
            sss.move("arm_left","side", False)
            sss.move("arm_right","side", False)
            sss.set_light("light_base","cyan", False)
            sss.set_light("light_torso","cyan", False)
            handle_say.wait()

            # sync


            sys.exit() 			
            n = n+1

        if n == 2: 
            sss.say(["One of my technical highlights is the modularity. Let me explain my different modules."])
            # :: 1.Explaing modules(base)
            rospy.loginfo("Explaining modules(base)")       
            sss.say(["I consist of 4 elementary parts: base, torso, arms and head, i'll start with my base."], False)
            sss.move("arm_right", "point2base")
            sss.say(["I can move forward and backward"], False)
            sss.move_base_rel("base", [-0.1, 0, 0])
            sss.move_base_rel("base", [0.1, 0, 0])
            sss.say(["or sideways"])
            sss.move_base_rel("base", [0, 0.1, 0])
            sss.move_base_rel("base", [0, -0.1, 0])
            sss.move_base_rel("base", [0, -0.1, 0], False)
            sss.say(["and back."], False)
            sss.move_base_rel("base", [0, 0.1, 0])
            sss.say(["I am also capable to turn on the spot like this"])
            sss.move_base_rel("base", [0, 0, 0.5])
            sss.move_base_rel("base", [0, 0, 0.5])
            sss.move_base_rel("base", [0, 0, -0.5])
            sss.move_base_rel("base", [0, 0, -0.5])
            sss.move_base_rel("base", [0, 0, -0.5])
            sss.move_base_rel("base", [0, 0, 0.5])
            sss.say(["And, of course, i can combine the movements"])
            sss.say(["Using my safety laser scanners in the base I can safely navigate between humans."])  

            n = n+1
        
        if n == 2:
            # :: 2.Explain modules (torso)
            rospy.loginfo("Explaining modules(torso)")
            sss.move("arm_right","point2chest")
            handle_drawtorso = sss.move("arm_right","draw_torso", False)
            sss.say(["The next module is my torso, you have already seen the lights i can change and use to interact with you"])
            sss.move("arm_right","point2camera")
            sss.say(["Here i have 3d cameras helping me to navigate and recognize obstacles "])

            n = n+1
        
        if n == 3:
			# :: 3.Explain modules (arms)
            rospy.loginfo("Explaining modules(arms)")
            sss.set_light("light_base","yellow")
            sss.set_light("light_torso","yellow")
            sss.move("arm_right", [[1.5, 0, 0, 0, 0, 0, 0]], False)
            sss.move("arm_left", [[-1.5, 0, 0, 0, 0, 0, 0]])
            sss.say(["as you can see, i have two arms, which i can use indepently"], False)
            r_handle = sss.move("arm_right", [[1.5, 0.3, 0, 0, 0, 0, 0]], False)
            sss.move("arm_left", [[-1.5, 0.3, 0, 0, 0, 0, 0]])           
            r_handle.wait()
            sss.move("arm_right", [[1.5, -0.3, 0, 0, 0, 0, 0]], False)
            sss.move("arm_left", [[-1.5, -0.3, 0, 0, 0, 0, 0]])
            
            sss.say(["Both arms consist of 7 independet joints, allowing me to perform complex movements"], False)

            rospy.loginfo("Beginning arena-like wave")
            sss.move("arm_right", [[1.5, 0, 0, 0, 0, 0, 0]], False)
            sss.move("arm_left", [[-1.5, 0, 0, 0, 0, 0, 0]])
            sss.move("arm_right", [[1.5, 0, 0, 0, 0, 0.5, 0]])
            sss.move("arm_right", [[1.5, -0.3, 0, 1, 0, -0.2, 0]])

            sss.move("arm_left", [[-1.5, 0.3, 0, -1, 0, 0.2, 0]], False)
            sss.move("arm_right", [[1.5, 0.3, 0, -1, 0, 0.2, 0]])

            sss.move("arm_left", [[-1.5, -0.3, 0, 1, 0, -0.2, 0]], False)
            sss.move("arm_right", [[1.5, -0.3, 0, 1, 0, -0.2, 0]])

            sss.move("arm_right", "side", False)
            handle_lastmv = sss.move("arm_left", "side")
            handle_lastmv.wait()
            
            # :: 3.1.Hands
            #sss.move("arm_right", "carry", False)
            #sss.move("arm_left", "carry", False)
            sss.say(["Not only do i have arms, but also hands."])
            ## Open , close hands
            sss.move("gripper_right","open")
            sss.move("gripper_left","open")
            sss.say(["Combined with my 3D-Sensors i am able to recognize objects and grab or manipulate them"])
            sss.move("gripper_right","closed")
            sss.move("gripper_left","closed")
            sss.say(["So i could not only entertain you, but acctually help you carry stuff around in your appartment"])
            sss.say(["If you stay until the end of my presentation, maybe i will serve you some fresh cookies"])			
            
            sss.move("arm_right", "side", False)
            handle_lastmv = sss.move("arm_left", "folded")
            handle_lastmv.wait()

            n = n+1

        if n == 4:
            # :: 4.Explain modules(head)
            rospy.loginfo("Explaining modules(head)")
            sss.move("arm_right", "point2head", False)
            sss.say(["And finally, something i always forget, my head"])
            sss.set_mimic("mimic",["surprised",0,3])
            sss.move("arm_right", "side")
            sss.set_mimic("mimic","happy")
            sss.say(["To detect objects and recognize people i can move my sensor ring"])
            sss.move("sensorring","left")
            sss.move("sensorring","right")
            sss.move("sensorring","front")

        if n == 5:
            # :: 5.Software highlights
            sss.move("arm_right", "open", False)
            sss.move("arm_left", "open", False)
            sss.say(["Now that you have seen my hardware highlights, ill tell you something about my software"])
            sss.set_mimic("mimic",["blinking_right",0,2])
            sss.say(["I am shipped with open source drivers and powered by the Open Source Robot Operating System"])
            
            
            
        ## :: Final
        rospy.loginfo("Final/Exit scene reached")
        sss.say(["Now, my presentation has finally come to and end"])
        sss.say(["Thank you for your intereset"])
        sss.move_base_rel("base", [0, 0, 0.5])
        sss.say(["Thank you"])
        sss.move_base_rel("base", [0, 0, -1], False)
        sss.say(["Thank you for your attention"])
        sss.move_base_rel("base", [0, 0, 0.5])

        #Menu to select finishing action
        rospy.loginfo("------ Menu for exit scenes ------")
        rospy.loginfo("!!!DEPENTS FROM ENVIROMENT - BE CAREFUL!!!")
        rospy.loginfo("0 = arms folded")
        rospy.loginfo("1 = get cookies(HMI-2015)")
        while True:
            try:
                user_input=raw_input("Please select how to finish the presentation:")
                i=int(user_input)
                break
            except ValueError:
                rospy.loginfo("You didn't type a number, please try again")

        if i == 0:
            #Arms folded
            sss.move("arm_left", "folded", False)
            sss.move("arm_right", "folded")

        if i == 1:
            ## Find cookies, bring them to the viewers
            pass

        return "succeeded"

## -- State Machine 

class Explore(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,
            outcomes=['finished','failed'])
        with self:

#            smach.StateMachine.add('COB_INTRODUCTION_PREPARE',CobIntroductionPrepare(),
#                transitions={'succeeded':'COB_INTRODUCTION',
#                    'failed':'failed'})

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
