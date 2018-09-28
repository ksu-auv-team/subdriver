#!/usr/bin/env python

from StateMachine.sub import *

# define state Foo
class search_front(sub):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Found_Object','Not_Found_Object'])

    def execute(self, userdata):
    	self.init_state()
    	msg = self.init_joy_msg()
    	msg.axes[self.axes_dict['frontback']] = .4
        self.depth_hold(self.current_state_start_altitude)

    	#return 'Not_Found_Object' # Debug purposes only!

    	while(1):
            self.joy_pub.publish(msg)
            if gbl.get_box_of_class(gbl.boxes, gbl.current_target):
    			if self.search_frames_seen <= 2:
    				self.search_frames_seen += 1
    			else:
    				return "Found_Object" # Transitions to TRACK_GATE

            elif (rospy.get_time() - self.current_state_start_time) > 5:
    			self.search_frames_seen = 0
    			return "Not_Found_Object" # Transitions to SEARCH_LEFT_GATE

            else:
    			self.search_frames_seen = 0
        
