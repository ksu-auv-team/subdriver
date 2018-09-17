#!/usr/bin/env python

from sub import *

# define state Foo
class surface(sub):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Surfaced'])

    def execute(self, userdata):
        rospy.loginfo('Executing state SURFACE')
        return 'Surfaced'