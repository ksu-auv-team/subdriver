#!/usr/bin/env python2

from StateMachine.sub import *
from search_recenter import *

'''
Implements search moving forward for the octagon/surfacing tasks
Different from the other searches because it needs to look for
multiple objects, the coffin and the octagon
'''

# define state search_recenter
class search_recenter_octagon(sub):
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
            #will need to change this to multiple targets
    		if gbl.get_box_of_class(gbl.boxes, gbl.current_target):
    			if self.search_frames_seen <= 2:
    				self.search_frames_seen += 1
    			else:
    				return "Found_Object" # Transitions to track_octagon

    		elif (rospy.get_time() - self.current_state_start_time) > 2:
    			self.search_frames_seen = 0
    			return "Not_Found_Object" # Transitions to search_front_octagon

    		else:
    			self.search_frames_seen = 0
				
			rospy.sleep(gbl.sleep_time)

