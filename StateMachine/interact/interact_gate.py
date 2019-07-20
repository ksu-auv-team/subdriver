#!/usr/bin/env python

from StateMachine.sub import *

# define state interact_gate
class interact_gate(sub):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Through_Gate'])

    def execute(self, userdata):
        self.init_state()

        msg = self.init_joy_msg()
        msg.axes[self.axes_dict['frontback']] = 0.7
        

        rospy.loginfo("Charging forward for 5 seconds")
        
        while rospy.get_time() < (self.current_state_start_time + 5):
            self.joy_pub.publish(msg)
            rospy.sleep(gbl.const.const.SLEEP_TIME)

        gbl.current_target = self.class_dict['pole']

        return 'Through_Gate' # Transitions to SEARCH_FRONT_POLE


    def log(self):
        rospy.loginfo('Executing state INTERACT_GATE')