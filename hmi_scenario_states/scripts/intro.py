#!/usr/bin/python
import roslib
roslib.load_manifest('hmi_scenario_states')
import rospy
import smach
import smach_ros
import sys

import random

from std_srvs.srv import Trigger

from simple_script_server import *
sss = simple_script_server()

def wave(side):
    sss.move("arm_" + side,"wave1")
    sss.move("arm_" + side,"wave2")
    sss.move("arm_" + side,"wave1")
    sss.move("arm_" + side,"wave2")
    sss.move("arm_" + side,"side", False)

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
        res.success = True
        return res

    def continue_cb(self, req):
        print "continue intro"
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
            sss.say(["Hello and welcome to my presentation, my name is Care o bot. I am a mobile service robot build by Fraunhofer I. P. A. and Schunk."], False)
            sss.move_base_rel("base", [0, 0, -0.7], False)
            #sss.move("arm_right", "wave_hmi")
            wave("right")

            # sync

            sss.move_base_rel("base", [0, 0, 1.4])

            #sss.say(["I am a gentleman, so dont be afraid."], False)
            #sss.say(["Dont be afraid, i am a gentleman."])
            #sss.say(["I have a wide range of services. I can assist you at home or serve food and drinks in restaurants or hotels. In hospitals and care facilities I can give support in various delivery tasks."], False) 
            
            sss.say(["Dont be afraid, i am a gentleman. I have a wide range of services. At home I could assist you and serve drinks or work in hotels or restaurants. In hospitals and care facilities there are a lot of delivery task that I could take over."], False)
            wave("left")
            
            #TODO show renderings on head display while explaining application domanis
            #handle_arm_left = sss.move("arm_left", "wave_hmi", False)
            #wave("left")

            #sss.move_base_rel("base", [0, 0, -0.5])
            sss.move("base","middle",mode="linear")
            #handle_arm_left.wait()
            
            # sync
            rospy.sleep(1.0)
            n = n+1

        if n == 1:
            # Explain lights & display
            sss.set_mimic("mimic", ["laughing",0,2], False)
            sss.move("arm_right","point2head")
            sss.set_mimic("mimic", ["blinking_right",0,1], False)
            sss.say(["When I interact with humans. I can use my eyes to express my mood."])
            #sss.set_mimic("mimic","laughing")
            rospy.sleep(0.5)
            
            sss.move("arm_right","point2base", False)
            sss.say(["By chaning the color in my torso and base I can provide information about my current status."], False)
            sss.move("arm_left","point2base")
            sss.set_mimic("mimic","happy")

            # sync
            handle_light = sss.set_light("light_base","yellow", False)
            sss.set_light("light_torso","yellow", False)
            sss.set_mimic("mimic", ["busy",0,1], False)
            sss.say(["If I light up in yellow that means that I am moving my base or my arms, so please pay attention."], False)
            sss.move_base_rel("base", [0, 0, -0.50])
            #rospy.sleep(0.5)
            
            sss.move("base","middle",mode="linear")
            #sss.move_base_rel("base", [0, 0, 0.50], False)
            #rospy.sleep(3)

            # sync
            handle_light.wait()
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
            rospy.sleep(1)
            n = n+1

        if n == 2: 
            # :: 1.Explaing modules(base)
            rospy.loginfo("Explaining modules(base)")
            sss.say(["One of my technical highlights, is the modularity. Let me explain my different modules."])
            handle_say = sss.say(["I consist of 4 elementary parts. base. torso. arms and head."])
            sss.set_light("light_torso", "yellow", False)
            sss.set_light("light_base", "yellow")
            handle_say.wait()
            
            sss.say(["I will start with my base."], False)
            sss.move("arm_right", "point2base")            
            
            sss.say(["I can move forward."], False)
            #sss.move("arm_right", "side",False)
            sss.move_base_rel("base", [0.4, 0, 0])
            sss.say(["and backward too."], False)
            sss.move("base","middle",mode="linear")
            sss.say(["Sideways is easy for me."], False)
            #sss.say(["Ha ha your car can not do this. Right?"], False)
            sss.move_base_rel("base", [0, 0.4, 0])
            sss.move_base_rel("base", [0, -0.8, 0])
            sss.move("base","middle",mode="linear")
            #sss.say(["and back."], False)
            sss.say(["I am also capable to turn on the spot like this."], False)
            sss.move_base_rel("base", [0, 0, 0.6])
            rospy.sleep(0.5)
            #sss.move_base_rel("base", [0, 0, -0.3])
            sss.move("base","middle",mode="linear")
            sss.set_light("light_base", "cyan", False)
            sss.set_light("light_torso", "cyan")

            rospy.sleep(1.0)
            sss.say(["Using my safety laser scanners in the base I can safely navigate between moving humans and objects."], False) #therefore we can use the same workspace and environment together."])
            sss.move("arm_right", "point2laser")
            rospy.sleep(1.0)
            sss.move("arm_right", "point2base")
            
            #sync
            rospy.sleep(1)
            n = n+1
        
        if n == 3:
            # :: 2.Explain modules (torso)
            rospy.loginfo("Explaining modules(torso)")
            sss.set_light("light_base", "yellow", False)
            sss.set_light("light_torso", "yellow")
            rospy.sleep(1.0)
            sss.say(["The next module is my torso. In the future I will be able to move it."], False)
            #sss.move("arm_right","point2chest")
            #sss.move("arm_right","draw_torso", False)
            #sss.say(["My torso also is agile and can swing around, allowing me to grab objects from the floor or shelfs."])
            #sss.move("arm_right","point2camera")
            sss.move("arm_right","point2torso_low")
            sss.move("arm_right","point2torso_up")
            sss.move("arm_right","point2torso_low")            
            sss.set_light("light_base","cyan", False)
            sss.set_light("light_torso", "cyan", False)
            sss.say(["Here i have 3d cameras helping me to navigate and recognize obstacles."])
            
            #sync
            #rospy.sleep(1)
            n = n+1
            #sys.exit()

        if n == 4:
			# :: 3.Explain modules (arms)
            rospy.loginfo("Explaining modules(arms)")
            sss.set_light("light_base","yellow")
            sss.set_light("light_torso","yellow")

            #folding_right_handle = sss.move("arm_right",["side", "reverse_folded", "side", "folded"], False)
            #folding_left_handle = sss.move("arm_left",["side", "reverse_folded", "side", "folded"], False) 
            
            sss.say(["The best part of me: My two flexible arms. I can use them independently or synchronized."],False)
            rospy.loginfo("Beginning arena-like wave")
            sss.move("arm_right", [[1.5, 0, 0, 0, 0, 0, 0]],False)
            sss.move("arm_left", [[-1.5, 0, 0, 0, 0, 0, 0]])
            sss.say(["My arms and joints were developed in cooperation with Schunk and follow the shape of human arms."],False)
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

            sss.say(["At the end of my arms, I have grippers."])
            sss.set_light("light_base","yellow", False)
            sss.set_light("light_torso","yellow")
            sss.say(["Combined with my in-gripper 3D-Camera i am able to recognize objects and manipulate or grab them."],False)
            sss.move_base_rel("base",[0,0,-0.3],False)
            sss.move("arm_right",[[1.7642137145009082, 1.2919974320813223, -0.82794929056107, -1.8777473823431394, 0.0, 0.0, 0.0]])
            sss.move("gripper_right","open")
            
            sss.move("arm_right",[[1.7642137145009082, 1.2919799787888024, -1.6467705091342097, -1.8777473823431394, 0.0, 0.0, 0.0]],False)
            sss.move_base_rel("base",[0,0,0.6], False)
            sss.move("gripper_right","home")
            sss.move("gripper_right","open")
            #sss.move("arm_right",[[1.2, 1.2, -1.0472, -1.6581, -1.0472, -0.6981, 0.5]], False)
            #sss.move("gripper_left","open")

            rospy.sleep(1.0)
            sss.say(["As you can see, I can not only entertain, but actually support your work or lifestyle by grasping and carrying small objects."], False)
            sss.move("gripper_right","home", False)
            sss.move("base","middle",mode="linear")
            #sss.move_base_rel("base",[0,0,-0.3], False)
            sss.move("arm_right","side")
            
            #sss.move("gripper_left","home")
            sss.set_light("light_base","cyan", False)
            sss.set_light("light_torso","cyan")
            
            #sync
            rospy.sleep(1)
            n = n+1

        if n == 5:
            # :: 4.Explain modules(head)
            rospy.loginfo("Explaining modules(head)")
            sss.set_light("light_torso","yellow", False)
            sss.set_light("light_base", "yellow")
            sss.say(["And finally, something i always forget, my head."],False)
            handle_arm = sss.move("arm_right", "point2head")
            sss.set_mimic("mimic",["surprised",0,3])
            #sss.move("arm_right", "side")
            sss.set_mimic("mimic","happy")
            #rospy.sleep(3)
            
            sss.move("arm_right", "point2sensorring", False)
            sss.say(["In the black ring there can be various sensors. At the moment I have a 3D camera to detect objects and recognize people."])
            sss.move("arm_right", "point2head")
            #sss.move("sensorring","left")
            #sss.move("sensorring","right")
            #sss.move("sensorring","front")
            #rospy.sleep(4)
            sss.move("arm_right","side")
            sss.set_light("light_torso","cyan", False)
            sss.set_light("light_base","cyan", False)

            #sync
            rospy.sleep(1)
            n = n+1

        if n == 6:
            # :: 5.Software highlights
            sss.say(["Now that you have seen my hardware highlights, I will tell you something about my software."], False)
            sss.set_mimic("mimic",["blinking_right",0,2])
            sss.set_light("light_torso","yellow", False)
            sss.set_light("light_base","yellow", False)
            sss.move_base_rel("base",[0.5,0.5,0])
            sss.say(["I am powered by the Open Source Robot Operating System."], False)
            sss.move_base_rel("base",[0,-1.0,0])
            sss.say(["This architecture allows to upgrade my software and learn new skills."], False)
            sss.set_mimic("mimic",["yes",0,3])
            sss.move("base","middle",mode="linear")
            #sss.move_base_rel("base",[-0.3,0,0], False)
            sss.say(["You can buy me as a complete package with customized module configurations, for example special sensors or different degrees of freedom."])
            #sss.say(["For further informations please visit my website care minus o minus bot minus four dot com"])
            
            sss.set_light("light_torso","cyan", False)
            sss.set_light("light_base","cyan", False)
            #sync
            #rospy.sleep(1)
            
        ## :: Final

        rospy.loginfo("Final/Exit scene reached")
        sss.set_mimic("mimic","happy", False)
        sss.say(["Now, my presentation has come to an end."])

        sss.set_light("light_torso", "yellow", False)
        sss.set_light("light_base","yellow")
        sss.say(["Thank you for your interest, please ask my human colleagues for further information or take one of my bussiness cards from the desk."], False)
        sss.move_base_rel("base", [0, 0, 0.5], False)
        #sss.move("arm_right", "wave_hmi",False)
        wave("right")
        sss.say(["Enjoy your stay."], False)
        #sss.move("arm_left", "wave_hmi",False)
        wave("left")
        #rospy.sleep(2)
        sss.set_mimic("mimic","happy")
        sss.say(["Goodbye, and have a nice day."])
        sss.move_base_rel("base", [0, 0, -0.8], False)
        #sss.say(["Thank you for your attention"])
        sss.move("arm_right", "side",False)
        sss.move("arm_left", "side",False)
        sss.move("base","middle",mode="linear")
        #sss.move_base_rel("base", [0, 0, 0.5])
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
