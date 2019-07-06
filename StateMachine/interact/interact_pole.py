#!/usr/bin/env python

from StateMachine.sub import *

# define state interact_pole
class interact_pole(sub):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Around_Pole'])

    def execute(self, userdata):
        self.init_state()

        msg = self.init_joy_msg()
        msg.axes[self.axes_dict['frontback']] = 0.7
        msg.axes[self.axes_dict['vertical']] = self.depth_hold()

        rospy.loginfo("Charging forward for 10 seconds")
        
        while rospy.get_time() > (gbl.run_start_time + 15):
            self.joy_pub.publish(msg)
            rospy.sleep(gbl.sleep_time)

        gbl.current_target = None

        return 'Around_Pole' # Transitions to SEARCH_FRONT_DICE


    def log(self):
        rospy.loginfo('Executing state INTERACT_POLE')