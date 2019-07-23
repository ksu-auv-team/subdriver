#!/usr/bin/env python

from StateMachine.sub import *

# define state search_recenter
class Search_Recenter(Sub):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Found_Object','Not_Found_Object'])

    def execute(self, userdata):
        self.init_state()
        msg = self.init_joy_msg()
        msg.axes[const.AXES['rotate']] = -.05
        

        if(gbl.debug):
            return "Found_Object" # DEBUG purposes only

        while(1):
            self.publish(msg)
            if self.get_box_of_class(gbl.detections, gbl.current_target):
                if self.search_frames_seen <= 2:
                    self.search_frames_seen += 1
                else:
                    return "Found_Object" # Transitions to TRACK_GATE

            elif (rospy.get_time() - self.current_state_start_time) > 2:
                self.search_frames_seen = 0
                return "Not_Found_Object" # Transitions to SEARCH_FRONT_GATE

            else:
                self.search_frames_seen = 0
                
            rospy.sleep(const.SLEEP_TIME)

