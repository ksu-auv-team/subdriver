#!/usr/bin/env python

from StateMachine.sub import *

# define state search_recenter
class search_recenter(sub):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Found_Object','Not_Found_Object'])

    def execute(self, userdata):
    	self.init_state()
    	msg = self.init_joy_msg()
    	msg.axes[self.axes_dict['rotate']] = -.2
        msg.axes[self.axes_dict['vertical']] = self.depth_hold()

    	return 'Found_Object' # Debug purposes only!

    	while(1):
    		self.joy_pub.publish(msg)
    		if gbl.get_box_of_class(gbl.boxes, gbl.current_target):
    			if self.search_frames_seen <= 2:
    				self.search_frames_seen += 1
    			else:
    				return "Found_Object" # Transitions to TRACK_GATE

    		elif (rospy.get_time() - self.current_state_start_time) > 2:
    			self.search_frames_seen = 0
    			return "Not_Found_Object" # Transitions to SEARCH_FRONT_GATE

    		else:
    			self.search_frames_seen = 0
				
			rospy.sleep(gbl.sleep_time)

