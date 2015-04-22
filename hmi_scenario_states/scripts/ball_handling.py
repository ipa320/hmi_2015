#!/usr/bin/python
import rospy
import smach
import smach_ros
import tf
import sys
import copy

from cob_srvs.srv import Trigger, TriggerRequest, TriggerResponse

from simple_script_server import *
sss = simple_script_server()


def grasp_ball(req):
    print "grasp"
    my_client_pre_ball = rospy.ServiceProxy('/scenario/po1', Trigger)
    my_client_hold_ball = rospy.ServiceProxy('/scenario/rec1', Trigger)

    #wait_for_services
    if not wait_for_service('/scenario/po1', 5.0): #pre_ball_start
        res = TriggerResponse()
        res.success.data = False
        return res
    if not wait_for_service('/scenario/rec1', 5.0): #hold_ball
        res = TriggerResponse()
        res.success.data = False
        return res
    
    rospy.loginfo("All Services available!")

    move_gripper("gripper_left", "home")
    move_gripper("gripper_right", "home")

    handle_arm_left = sss.move("arm_left","side",False)
    handle_arm_right = sss.move("arm_right","side",False)
    handle_arm_left.wait()
    handle_arm_right.wait()
    
    sss.set_light("light_base","cyan")
    sss.set_light("light_torso","cyan")
    
    rospy.loginfo("Moving to PreBall")
    if not call_service(my_client_pre_ball):
        rospy.logerr("Movement to PreBall failed!")
        res = TriggerResponse()
        res.success.data = False
        return res
    
    move_gripper("gripper_left", "spread")
    move_gripper("gripper_right", "spread")
    
    rospy.loginfo("Moving to HoldBall")
    if not call_service(my_client_hold_ball):
        rospy.logerr("Movement to HoldBall failed!")
        res = TriggerResponse()
        res.success.data = False
        return res
    
    res = TriggerResponse()
    res.success.data = True
    return res

#def grasp_ball(req):
#    print "grasp"
#    handle_arm_left = sss.move("arm_left","side",False)
#    handle_arm_right = sss.move("arm_right","side",False)
#    handle_arm_left.wait()
#    handle_arm_right.wait()
#
#    handle_arm_left = sss.move("arm_left","pre_ball",False)
#    handle_arm_right = sss.move("arm_right","pre_ball",False)
#    move_gripper("gripper_left", "spread")
#    move_gripper("gripper_right", "spread")
#    handle_arm_left.wait()
#    handle_arm_right.wait()
#
#    handle_arm_left = sss.move("arm_left","hold_ball",False)
#    handle_arm_right = sss.move("arm_right","hold_ball",False)
#    handle_arm_left.wait()
#    handle_arm_right.wait()
#    
#    res = TriggerResponse()
#    res.success.data = True
#    return res

def release_ball(req):
    print "release ball"
    sss.set_light("light_base","blue")
    sss.set_light("light_torso","blue")
    rospy.sleep(1.0)
    sss.set_light("light_base","cyan")
    sss.set_light("light_torso","cyan")
    rospy.sleep(1.0)
    sss.set_light("light_base","blue")
    sss.set_light("light_torso","blue")
    rospy.sleep(1.0)
    sss.set_light("light_base","cyan")
    sss.set_light("light_torso","cyan")
    rospy.sleep(1.0)

    #handle_arm_left = sss.move("arm_left","pre_ball",False)
    #handle_arm_right = sss.move("arm_right","pre_ball",False)
    #handle_arm_left.wait()
    #handle_arm_right.wait()

    handle_arm_left = sss.move("arm_left","side",False)
    handle_arm_right = sss.move("arm_right","side",False)
    move_gripper("gripper_left", "home")
    move_gripper("gripper_right", "home")
    handle_arm_left.wait()
    handle_arm_right.wait()

    res = TriggerResponse()
    res.success.data = True
    return res

def move_gripper(component_name, pos):
    error_code = -1
    counter = 0
    while not rospy.is_shutdown() and error_code != 0:
        print "trying to move", component_name, "to", pos, "retries: ", counter
        handle = sss.move(component_name, pos)
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


def wait_for_service(srv_name, timeout):
    try:
        rospy.wait_for_service(srv_name, timeout)
        return True
    except rospy.ROSException, e:
        rospy.logerr("Service %s not available within timout!" %srv_name)
        return False
    except rospy.ROSInterruptException, e:
        rospy.logerr("InterruptRequest received")
        return False

def call_service(proxy):
    try:
        req = TriggerRequest()
        #print req
        res = proxy(req)
        #print res
        if res.success.data:
            rospy.loginfo("Service call successful: %s"%res.error_message.data)
            return True
        else:
            rospy.logerr("Service not successful: %s"%res.error_message.data)
            return False
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        return False


## -- Initiation
class BallHandlingInit(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
        self.client_pre_ball = rospy.ServiceProxy('/scenario/po1', Trigger)
        self.client_hold_ball = rospy.ServiceProxy('/scenario/rec1', Trigger)

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
        
        #wait_for_services
        if not wait_for_service('/scenario/po1', 5.0): #pre_ball_start
            return "failed"
        if not wait_for_service('/scenario/rec1', 5.0): #hold_ball
            return "failed"
        if not wait_for_service('/scenario/rec2', 5.0): #z
            return "failed"
        if not wait_for_service('/scenario/rec3', 5.0): #roll
            return "failed"
        if not wait_for_service('/scenario/rec4', 5.0): #cross
            return "failed"
        if not wait_for_service('/scenario/rec5', 5.0): #circ8
            return "failed"
        
        rospy.loginfo("All Services available!")
        
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
        
        rospy.loginfo("Moving to PreBall")
        if not call_service(self.client_pre_ball):
            rospy.logerr("Movement to PreBall failed!")
            return "failed"
        
        move_gripper("gripper_left", "spread")
        move_gripper("gripper_right", "spread")
        
        rospy.loginfo("Moving to HoldBall")
        if not call_service(self.client_hold_ball):
            rospy.logerr("Movement to HoldBall failed!")
            return "failed"
        
        return "succeeded"


class BallHandlingExecute(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
        
        rospy.loginfo("BallHandlingExecute: INIT")
        
        self.client_z = rospy.ServiceProxy('/scenario/rec2', Trigger)
        self.client_roll = rospy.ServiceProxy('/scenario/rec3', Trigger)
        self.client_cross = rospy.ServiceProxy('/scenario/rec4', Trigger)
        self.client_circ8 = rospy.ServiceProxy('/scenario/rec5', Trigger)
        
        rospy.loginfo("...done")

            
    def execute(self, userdata):
    
        rospy.loginfo("BallHandlingExecute: EXECUTE")
        
        rospy.loginfo("Starting Movement Z")
        if not call_service(self.client_z):
            rospy.logerr("Movement Z failed!")
            return "failed"
        
        rospy.loginfo("Starting Movement CROSS")
        if not call_service(self.client_cross):
            rospy.logerr("Movement CROSS failed!")
            return "failed"
        
        rospy.loginfo("Starting Movement CIRC8")
        if not call_service(self.client_circ8):
            rospy.logerr("Movement CIRC8 failed!")
            return "failed"
            
        rospy.loginfo("All Movements successfull! BallHandling finished!")
        return "succeeded"


class CheckForStop(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['stop','continue'])
        self.stop_requested = False
        rospy.Service('/hmi/stop_ball', Trigger, self.stop_cb)
        rospy.Service('/hmi/continue_ball', Trigger, self.continue_cb)
        
        #lehmann
        rospy.Service('/hmi/grasp_ball', Trigger, grasp_ball)
        rospy.Service('/hmi/release_ball', Trigger, release_ball)
            
    def execute(self, userdata):
        if self.stop_requested:
            self.stop_requested = False
            return "stop"
        else:
            return "continue"

    def stop_cb(self, req):
        print "stop ball"
        self.stop_requested = True
        res = TriggerResponse()
        res.success.data = True
        return res

    def continue_cb(self, req):
        print "continue ball"
        self.stop_requested = False
        res = TriggerResponse()
        res.success.data = True
        return res

class BallHandlingFinalize(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','failed'])

    def execute(self, userdata):

        rospy.sleep(3) # time for removing the ball

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


## -- State Machine 

class BallHandling(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,outcomes=['finished','failed'])
        with self:

            smach.StateMachine.add('INIT',BallHandlingInit(),
                transitions={'succeeded':'CHECK_FOR_STOP',
                             'failed':'failed'})

            smach.StateMachine.add('CHECK_FOR_STOP',CheckForStop(),
                transitions={'stop':'FINALIZE',
                             'continue':'EXECUTE'})

            smach.StateMachine.add('EXECUTE',BallHandlingExecute(),
                transitions={'succeeded':'CHECK_FOR_STOP',
                             'failed':'failed'})

            smach.StateMachine.add('FINALIZE',BallHandlingFinalize(),
                transitions={'succeeded':'finished',
                             'failed':'failed'})


if __name__=='__main__':
    rospy.init_node('ball_handling')
    sm = BallHandling()
    sis = smach_ros.IntrospectionServer('sm', sm, 'SM')
    sis.start()
    outcome = sm.execute()
    rospy.spin()
    sis.stop()
