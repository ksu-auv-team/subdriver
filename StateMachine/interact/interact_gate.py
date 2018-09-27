#!/usr/bin/env python

from StateMachine.sub import *

# define state Foo
class interact_gate(sub):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Through_Gate'])

    def execute(self, userdata):
    	self.init_state()

    	msg = self.init_joy_msg()
    	msg.axes[self.axes_dict['frontback']] = 0.7
    	self.joy_pub.publish(msg)
    	self.depth_hold(self.current_state_start_altitude)

    	rospy.loginfo("Charging forward for 10 seconds")

    	rospy.sleep(10)

    	gbl.current_target = None

        return 'Through_Gate'


	def log(self):
		rospy.loginfo('Executing state INTERACT_GATE')