#!/usr/bin/env python2

from StateMachine.sub import *
from search_recenter import *

'''
Implements search moving forward for the octagon/surfacing tasks
Different from the other searches because it needs to look for
multiple objects, the coffin and the octagon
'''

# define state search_recenter
class search_recenter_octagon(Sub):
    def __init__(self):
        smach.State.__init__(self, outcomes=['object_found','object_not_found'])

    def execute(self, userdata):
        self.init_state()
        msg = self.init_joy_msg()
        msg.axes[const.AXES['rotate']] = -.2

        return 'object_found' # Debug purposes only!

        while(1):
            self.joy_pub.publish(msg)
            #will need to change this to multiple targets
            if self.get_box_of_class(gbl.detections_bottom, gbl.current_target):
                if self.search_frames_seen <= 2:
                    self.search_frames_seen += 1
                else:
                    return "object_found" # Transitions to track_octagon

            elif abs(self.angle_diff(gbl.heading, gbl.state_heading)) < 5:
                self.search_frames_seen = 0
                return "object_not_found" # Transitions to search_front_octagon

            else:
                self.search_frames_seen = 0
                
            rospy.sleep(const.SLEEP_TIME)

