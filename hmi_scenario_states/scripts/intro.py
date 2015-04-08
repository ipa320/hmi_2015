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

def wave(side):
    sss.move("arm_" + side,"wave1")
    sss.move("arm_" + side,"wave2")
    sss.move("arm_" + side,"wave1")
    sss.move("arm_" + side,"wave2")
    sss.move("arm_" + side,"side")

def move_gripper(component_name, pos):
    error_code = -1
    counter = 0
    while not rospy.is_shutdown() and error_code != 0:
        print "trying to move", component_name, "to", pos, "retries: ", counter
        handle = sss.move(component_name, pos)
        error_code = handle.get_error_code()
        if counter > 100:
            rospy.logerr(component_name + "does not work any more. retries: " + str(counter))
            sss.set_light("light_torso","red")
            return False
        counter += 1
    return True

class Init(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','failed'],
            output_keys=['stop_requested'])

    def execute(self, userdata):

        sss.move("base","middle",mode="linear")

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

        sss.set_light("light_base","cyan")
        sss.set_light("light_torso","cyan")

        userdata.stop_requested = False
        return "succeeded"

class CheckForStop(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['stop','continue'])
        self.stop_requested = False
        rospy.Service('/hmi/stop_intro', Trigger, self.stop_cb)
        rospy.Service('/hmi/continue_intro', Trigger, self.continue_cb)
            
    def execute(self, userdata):
        if self.stop_requested:
            self.stop_requested = False
            return "stop"
        else:
            return "continue"

    def stop_cb(self, req):
        print "stop intro"
        self.stop_requested = True
        res = TriggerResponse()
        res.success.data = True
        return res

    def continue_cb(self, req):
        print "continue intro"
        self.stop_requested = False
        res = TriggerResponse()
        res.success.data = True
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


## -- main script
class CobIntroduction(smach.State):
    def __init__(self, debug = False):
        smach.State.__init__(self, 
            outcomes=['succeeded','failed'])
        self.debug = debug
			
    def execute(self, userdata):
        n = 0
        # :: Menue to select scenes
        while self.debug:
            rospy.loginfo("------ Menu ------")
            rospy.loginfo("0 = Introduction")
            rospy.loginfo("1 = Modules: lights & display")
            rospy.loginfo("2 = Modules: base")
            rospy.loginfo("3 = Modules: torso")
            rospy.loginfo("4 = Modules: arms")
            rospy.loginfo("5 = Modules: head")
            rospy.loginfo("6 = Software")
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
            # :: 0.Introduction
            sss.set_mimic("mimic","happy")
            rospy.loginfo("Beginning introduction of Care-O-bot 4-2 for HMI 2015")
            sss.move("arm_right", "side", False)
            sss.move("arm_left", "side")

            # sync
            sss.say(["Hello and welcome to my presentation, my name is Care o bot. I am a mobile service robot build by Fraunhofer I. P. A., in Stuttgart."], False)
            sss.move_base_rel("base", [0, 0, -0.78], False)
            rospy.sleep(0.5)
            #sss.move("arm_right", "wave_hmi")
            wave("right")

            # sync

            sss.say(["I am a gentleman, so dont be afraid."], False)
            sss.move_base_rel("base", [0, 0, 1.3], False)

            rospy.sleep(1)
            sss.say(["I have a wide range of services. I can assist you at home or serve food and drinks in restaurants or hotels. In hospitals and care facilities I can give support in various delivery tasks."], False) #TODO show renderings on head display while explaining application domanis
            #handle_arm_left = sss.move("arm_left", "wave_hmi", False)
            wave("left")
            
            rospy.sleep(8)

            sss.move_base_rel("base", [0, 0, -0.78])
            #handle_arm_left.wait()
            
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
            sss.set_light("light_base","yellow", False)
            sss.set_light("light_torso","yellow", False)
            handle_say = sss.say(["If I light up in yellow that means that I am moving my base or my arms, so please pay attention."], False)
            sss.move_base_rel("base", [0, 0, -0.35], False)
            sss.set_mimic("mimic", ["busy",0,1], False)
            rospy.sleep(1)
            sss.move("arm_right","point2base", False)

            handle_say.wait()
            
            sss.move_base_rel("base", [0, 0, 0.35], False)
            rospy.sleep(3)

            # sync
            sss.set_light("light_base","red", False)
            sss.set_light("light_torso","red", False)
            handle_say = sss.say(["If you see me in red, there is an error and I need help. But dont worry, I feel good right now, this is only to show you my colors. I hope you will never see me in an error state."], False)
            sss.set_mimic("mimic", ["confused",0,1], False)
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


            rospy.sleep(4)
            sss.say(["One of my technical highlights, is the modularity. Let me explain my different modules."])         			

            n = n+1

        if n == 2: 
            # :: 1.Explaing modules(base)
            rospy.loginfo("Explaining modules(base)")       
            handle_say = sss.say(["I consist of 4 elementary parts. base, torso, arms and head. I will start with my base."], False)
            sss.set_light("light_torso", "yellow", False)
            sss.set_light("light_base", "yellow")
            sss.move("arm_right", "point2base")
            handle_say.wait()
            sss.say(["I can move forward and backward."], False)
            sss.move("arm_right", "side",False)
            sss.move_base_rel("base", [0.1, 0, 0])
            sss.move_base_rel("base", [-0.1, 0, 0])
            sss.say(["or sideways."])
            sss.move_base_rel("base", [0, 0.1, 0])
            sss.move_base_rel("base", [0, -0.1, 0])
            sss.move_base_rel("base", [0, -0.1, 0])
            #sss.say(["and back."], False)
            sss.move_base_rel("base", [0, 0.1, 0])
            sss.say(["I am also capable to turn on the spot like this."], False)
            sss.move_base_rel("base", [0, 0, 0.5])
            sss.move_base_rel("base", [0, 0, -0.5])
            sss.set_light("light_base", "cyan", False)
            sss.set_light("light_torso", "cyan")

            rospy.sleep(0.5)
            sss.say(["Using my safety laser scanners in the base I can safely navigate between moving humans and objects."]) #therefore we can use the same workspace and environment together."])  
            rospy.sleep(4)
            n = n+1
        
        if n == 3:
            # :: 2.Explain modules (torso)
            rospy.loginfo("Explaining modules(torso)")
            sss.set_light("light_base", "yellow", False)
            sss.set_light("light_torso", "yellow")
            sss.say(["The next module is my torso, you have already seen the lights i can change and use to interact with you."], False)
            sss.move("arm_right","point2chest")
            sss.move("arm_right","draw_torso", False)
            #sss.say(["My torso also is agile and can swing around, allowing me to grab objects from the floor or shelfs."])
            sss.move("arm_right","point2camera")
            sss.set_light("light_base","cyan", False)
            sss.set_light("light_torso", "cyan", False)
            sss.say(["Here i have 3d cameras helping me to navigate and recognize obstacles."])
            rospy.sleep(4)
            n = n+1
           # sys.exit()

        if n == 4:
			# :: 3.Explain modules (arms)
            rospy.loginfo("Explaining modules(arms)")
            sss.set_light("light_base","yellow")
            sss.set_light("light_torso","yellow")

            #folding_right_handle = sss.move("arm_right",["side", "reverse_folded", "side", "folded"], False)
            #folding_left_handle = sss.move("arm_left",["side", "reverse_folded", "side", "folded"], False) 
            
            sss.say(["As you can see, i have two flexible arms. I can use them independently."],False)
            rospy.loginfo("Beginning arena-like wave")
            sss.move("arm_right", [[1.5, 0, 0, 0, 0, 0, 0]],False)
            sss.move("arm_left", [[-1.5, 0, 0, 0, 0, 0, 0]])
            sss.say(["My arms and joints were developed in cooperation with Schoonk and follow the shape of human arms."],False)
            sss.move("arm_right", [[1.5, -0.3, 0, 1, 0, -0.7, 0]])
            sss.say(["and thanks to sensors i am able to share the same workspace with humans."],False)
            sss.move("arm_left", [[-1.5, 0.3, 0, -1, 0, 0.7, 0]], False)
            sss.move("arm_right", [[1.5, 0.3, 0, -1, 0, 0.7, 0]])
            sss.say([" Both arms consist of 7 independent joints allowing me to perform complex movements."], False)
            sss.move("arm_left", [[-1.5, -0.3, 0, 1, 0, -0.7, 0]],False)
            sss.move("arm_right", [[1.5, -0.3, 0, 1, 0, -0.7, 0]])
            #folding_right_handle.wait()
            #folding_left_handle.wait()
            sss.move("arm_right","side",False)
            sss.move("arm_left","side")

            sss.set_light("light_base","cyan")
            sss.set_light("light_torso","cyan")
            rospy.sleep(1)

            sss.say(["Not only do i have arms, but also grippers."])
            sss.set_light("light_base","yellow", False)
            sss.set_light("light_torso","yellow")
            sss.move_base_rel("base",[0,0,-0.3])
            sss.say(["Combined with my in-gripper 3D-Camera i am able to recognize objects and manipulate or grab them."],False)
            sss.move("arm_right",[[1.7642137145009082, 1.2919974320813223, -0.82794929056107, -1.8777473823431394, 0.0, 0.0, 0.0]])

            sss.move("gripper_right","open")
            sss.move_base_rel("base",[0,0,0.6], False)
            #sss.move("arm_right",[[1.2, 1.2, -1.0472, -1.6581, -1.0472, -0.6981, 0.5]], False)
            #sss.move("gripper_left","open")
            sss.move("arm_right",[[1.7642137145009082, 1.2919799787888024, -1.6467705091342097, -1.8777473823431394, 0.0, 0.0, 0.0]],False)
            sss.say(["As you can see, I can not only entertain, but actually support your work or lifestyle by grasping and carrying small objects."])
            sss.move("gripper_right","home", False)
            sss.move_base_rel("base",[0,0,-0.3], False)
            sss.move("arm_right","side")
            #sss.move("gripper_left","home")
            sss.set_light("light_base","cyan", False)
            sss.set_light("light_torso","cyan")
            rospy.sleep(3)
            
            n = n+1

        if n == 5:
            # :: 4.Explain modules(head)
            rospy.loginfo("Explaining modules(head)")
            sss.set_light("light_torso","yellow", False)
            sss.set_light("light_base", "yellow")
            sss.say(["And finally, something i always forget, my head."],False)
            handle_arm = sss.move("arm_right", "point2head",False)
            sss.set_mimic("mimic",["surprised",0,3])
            #sss.move("arm_right", "side")
            sss.set_mimic("mimic","happy")
            rospy.sleep(3)
            handle_say = sss.say(["To detect objects and recognize people i can use my sensor ring allowing me to operate in dynamic human environments."], False)
            #sss.move("sensorring","left")
            #sss.move("sensorring","right")
            #sss.move("sensorring","front")
            handle_arm.wait()
            rospy.sleep(4)
            handle_say.wait()
            sss.move("arm_right","side")
            sss.set_light("light_torso","cyan", False)
            sss.set_light("light_base","cyan", False)

            n = n+1

        if n == 6:
            # :: 5.Software highlights
            sss.say(["Now that you have seen my hardware highlights, I will tell you something about my software."])
            sss.set_mimic("mimic",["blinking_right",0,2])
            sss.move_base_rel("base",[0.1,0,0], False)
            sss.say(["I am powered by the Open Source Robot Operating System."])
            sss.move_base_rel("base",[0,0,0.5], False)
            sss.say(["This architecture allows to upgrade my software and learn new skills."])
            sss.move_base_rel("base",[0,0,-0.5])
            sss.set_light("light_torso","cyan", False)
            sss.set_light("light_base","cyan", False)
            sss.set_mimic("mimic",["yes",0,3])
            sss.move_base_rel("base",[-0.1,0,0], False)
            sss.say(["You can buy me as a complete package with customized module configurations, for example special sensors or different degrees of freedom."])
            rospy.sleep(4)

            #sss.say(["For further informations please visit my website care minus o minus bot minus four dot com"])
                   
            
        ## :: Final

        rospy.loginfo("Final/Exit scene reached")
        sss.set_mimic("mimic",["happy", 0, 4])
        sss.say(["Now, my presentation has come to an end."])

        sss.set_light("light_torso", "yellow", False)
        sss.set_light("light_base","yellow")
        sss.say(["Thank you for your interest, please ask my human colleagues for further information or take one of my bussiness cards from the desk."])
        sss.move_base_rel("base", [0, 0, 0.5])
        #sss.move("arm_right", "wave_hmi",False)
        wave("right")
        #sss.move("arm_left", "wave_hmi",False)
        wave("left")
        rospy.sleep(2)
        sss.set_mimic("mimic","happy")
        sss.say(["Goodbye, and have a nice day."])
        sss.move_base_rel("base", [0, 0, -0.8])
        #sss.say(["Thank you for your attention"])
        sss.move("arm_right", "side",False)
        sss.move("arm_left", "side",False)
        sss.move_base_rel("base", [0, 0, 0.5])
        sss.set_light("light_torso","cyan",False)
        sss.set_light("light_base","cyan")


        return "succeeded"

## -- State Machine 

class Intro(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,
            outcomes=['finished','failed'])
        with self:

            smach.StateMachine.add('INIT',Init(),
                transitions={'succeeded':'CHECK_FOR_STOP',
                             'failed':'failed'})

            smach.StateMachine.add('CHECK_FOR_STOP',CheckForStop(),
                transitions={'stop':'FINALIZE',
                             'continue':'INTRO'})

            smach.StateMachine.add('INTRO',CobIntroduction(),
                transitions={'succeeded':'finished',
                    'failed':'failed'})

            smach.StateMachine.add('FINALIZE',Finalize(),
                transitions={'succeeded':'finished',
                             'failed':'failed'})


class SM(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,outcomes=['ended'])
        with self:
            smach.StateMachine.add('STATE',Intro(),
                transitions={'finished':'ended',
                    'failed':'ended'})

if __name__=='__main__':
    rospy.init_node('intro')
    sm = SM()
    sis = smach_ros.IntrospectionServer('sm', sm, 'SM')
    sis.start()
    outcome = sm.execute()
    rospy.spin()
    sis.stop()
