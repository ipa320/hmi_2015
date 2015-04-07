#!/usr/bin/python
import rospy
import smach
import smach_ros
import sys

from cob_srvs.srv import Trigger, TriggerResponse, TriggerRequest

# states
from roses import Roses
from ball_handling import BallHandling

## -- Initiation
class SelectScenario(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['pause', 'roses', 'intro', 'ball','failed'])

        rospy.Service('/hmi/select_pause', Trigger, self.trigger_pause_cb)
        rospy.Service('/hmi/select_roses', Trigger, self.trigger_roses_cb)
        rospy.Service('/hmi/select_intro', Trigger, self.trigger_intro_cb)
        rospy.Service('/hmi/select_ball', Trigger, self.trigger_ball_cb)
        
        # clients
        self.srv_stop_roses = rospy.ServiceProxy('/hmi/stop_roses', Trigger)
        self.srv_stop_intro = rospy.ServiceProxy('/hmi/stop_intro', Trigger)
        self.srv_stop_ball = rospy.ServiceProxy('/hmi/stop_ball', Trigger)

        self.next_scenario = "pause"

    def execute(self, userdata):
        if not self.wait_for_service("/hmi/stop_roses", 5.0):
            return 'failed'
        #if not self.wait_for_service("/hmi/stop_intro", 5.0):
        #    return 'failed'
        if not self.wait_for_service("/hmi/stop_ball", 5.0):
            return 'failed'
        rospy.sleep(3)
        
        return self.next_scenario

    def wait_for_service(self, srv_name, timeout):
        try:
            rospy.wait_for_service(srv_name, timeout)
            return True
        except rospy.ROSException, e:
            rospy.logerr("Service %s not available within timout!" %srv_name)
            return False
        except rospy.ROSInterruptException, e:
            rospy.logerr("InterruptRequest received")
            return False

    def trigger_pause_cb(self,req):
        self.next_scenario = "pause"
        self.stop_all_sub_state_machines()
        print "selected", self.next_scenario
        res = TriggerResponse()
        res.success.data = True
        return res

    def trigger_roses_cb(self,req):
        self.next_scenario = "roses"
        self.stop_all_sub_state_machines()
        print "selected", self.next_scenario
        res = TriggerResponse()
        res.success.data = True
        return res

    def trigger_intro_cb(self,req):
        self.next_scenario = "intro"
        self.stop_all_sub_state_machines()
        print "selected", self.next_scenario
        res = TriggerResponse()
        res.success.data = True
        return res

    def trigger_ball_cb(self,req):
        self.next_scenario = "ball"
        self.stop_all_sub_state_machines()
        print "selected", self.next_scenario
        res = TriggerResponse()
        res.success.data = True
        return res

    def stop_all_sub_state_machines(self):
        req = TriggerRequest()
        
        if not self.next_scenario == "roses":
            self.srv_stop_roses(req)
        if not self.next_scenario == "intro":
            self.srv_stop_intro(req)
        if not self.next_scenario == "ball":
            self.srv_stop_ball(req)


## -- State Machine 

class Hmi(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,
            outcomes=['finished','failed'])
        
        with self:

            smach.StateMachine.add('SELECT_SCENARIO',SelectScenario(),
                transitions={'pause':'SELECT_SCENARIO',
                    'roses':'ROSES',
                    'intro':'SELECT_SCENARIO', #FIXME this is still a dummy
                    'ball':'BALL',
                    'failed':'failed'})

            smach.StateMachine.add('ROSES',Roses(),
                transitions={'finished':'SELECT_SCENARIO',
                    'failed':'failed'})

            smach.StateMachine.add('BALL',BallHandling(),
                transitions={'finished':'SELECT_SCENARIO',
                        'failed':'failed'})


class SM(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,outcomes=['ended'])
        with self:
            smach.StateMachine.add('STATE',Hmi(),
                transitions={'finished':'ended',
                    'failed':'ended'})

if __name__=='__main__':
    rospy.init_node('hmi')
    sm = SM()
    sis = smach_ros.IntrospectionServer('sm', sm, 'SM')
    sis.start()
    outcome = sm.execute()
    rospy.spin()
    sis.stop()
