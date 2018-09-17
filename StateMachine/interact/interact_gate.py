#!/usr/bin/env python

from StateMachine.sub import *

# define state Foo
class interact_gate(sub):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Through_Gate'])

    def execute(self, userdata):
        rospy.loginfo('Executing state INTERACT_GATE')
        return 'Through_Gate'