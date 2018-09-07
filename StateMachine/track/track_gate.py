#!/usr/bin/env python

import rospy
import smach
import smach_ros

from StateMachine.sub import *

# define state Foo
class track_gate(sub):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Lost_Gate','Entered_Gate'])

    def execute(self, userdata):
        rospy.loginfo('Executing state TRACK_GATE')
        return 'Entered_Gate'