#!/usr/bin/env python

import rospy
import smach
import smach_ros

from sub import *

# define state Foo
class start(sub):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Not_Found_Gate', 'Found_Gate'])
        self.init_state()

    def execute(self, userdata):
    	self.run_start_time = rospy.get_time()

    	curr_msg = self.init_joy_msg()
    	curr_msg.axes[self.axes_dict['vertical']] = -1
    	curr_msg.axes[self.axes_dict['frontback']] = 1

    	self.joy_pub.publish(curr_msg)

    	if rospy.get_time() > (self.run_start_time + 15):
    		self.current_target = self.class_dict['start_gate']
        	if self.get_box_of_class(self.boxes, self.class_dict['start_gate']):
        		return 'Found_Gate' # Transitions to TRACK_GATE
        	else:
        		return 'Not_Found_Gate' # Transitions to SEARCH_FRONT_GATE

        return 'Not_Found_Gate' # Debug Purpuses Only!

    def log(self):
    	rospy.loginfo('Executing state START')