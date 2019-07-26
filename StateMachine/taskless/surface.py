#!/usr/bin/env python2

from StateMachine.sub import *

# define state surface
class Surface(Sub):
    def __init__(self):
        smach.State.__init__(self, outcomes=['surfaced'])

    def execute(self, userdata):
        rospy.loginfo('Executing state SURFACE')
        return 'Surfaced'
