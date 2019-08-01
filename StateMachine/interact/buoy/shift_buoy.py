#!/usr/bin/env python
from StateMachine.gbl import *
from StateMachine.const import *
from StateMachine.sub import *
import rospy

'''
State inheriting from sub that will shift from the first buoy to the second buoy.
'''

class Shift_Buoy(Sub):
    def __init__(self):
        smach.State.__init__(self, outcomes=['finished_shifting'])

    def execute(self, userdata):
        rospy.loginfo('Executing state SHIFT_BUOY')
        # At this point, the sub is stationary and facing the Buoy
        self.init_state()
        
        msg = self.init_joy_msg()            






