#!/usr/bin/env python

from StateMachine.sub import *

# define state interact_pole
class interact_pole(sub):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Around_Pole','Lost_Pole'])

    def execute(self, userdata):
        self.init_state()

        msg = self.init_joy_msg()
        msg.axes[self.axes_dict['vertical']] = self.depth_hold()
        
        num_turns = 0
        pole_on_right = False
        pole_on_left = False
        ''' WORK IN PROGRESS
        while num_turns < 3:
            while (not pole_on_right):
                box = self.get_box_of_class(gbl.boxes, gbl.current_target)
                msg.axes[self.axes_dict['leftright']] = -0.1

            while (not pole_on_left):
                msg.axes[self.axes_dict['rotate']] = 0.05

        '''
        gbl.current_target = self.class_dict['start_gate']

        return 'Around_Pole' # Transitions to SEARCH_FRONT_GATE


    def log(self):
        rospy.loginfo('Executing state INTERACT_POLE')