#!/usr/bin/env python

from StateMachine.sub import *

# define state Foo
class track_gate(sub):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Lost_Gate','Entered_Gate'])

    def execute(self, userdata):
    	self.init_state()
    	self.last_seen = rospy.get_time()

    	while(1):
    		msg = self.init_joy_msg()
    		box = gbl.get_box_of_class(gbl.boxes, gbl.current_target)
			msg.axes[self.axes_dict['vertical']] = gbl.depth_const

    		if (box != None) and box[1] > .3:
    			self.last_seen = rospy.get_time()
    			center = self.getCenter(box)
    			msg.axes[self.axes_dict['frontback']] = 0.4
    			
    			if center[0] < 0.45:
    				msg.axes[self.axes_dict['rotate']] = 0.1
    			elif center[0] > 0.55:
    				msg.axes[self.axes_dict['rotate']] = -0.1

				if center[1] < .45:
					if self.get_depth() > 1:
						msg.axes[self.axes_dict['vertical']] = gbl.depth_const + 0.2
					else: 
						msg.axes[self.axes_dict['vertical']] = gbl.depth_const
        		elif center[1] > .55:
        			msg.axes[self.axes_dict['vertical']] = gbl.depth_const - 0.2
            		

            	if box:
            		if self.getDistance(box[2], box[3], box[4], box[5]) > 0.4:
            			self.is_close = True

				self.joy_pub.publish(msg)


           	elif (rospy.get_time() - self.last_seen) > 2:
				msg.axes[self.axes_dict['frontback']] = 0
	    		self.joy_pub.publish(msg)


           		if self.is_close:
           			self.is_close = False
           			return "Entered_Gate" # Transitions to INTERACT_GATE
           		else:
           			rospy.logwarn("Lost tracking the gate for more than 2 seconds")
           			return "Entered_Gate" # DEBUG Purposes Only!
           			return "Lost_Gate" # Transitions to SEARCH_FRONT_GATE

		rospy.sleep(gbl.sleep_time)

	def log(self):
		rospy.loginfo('Executing state TRACK_GATE')