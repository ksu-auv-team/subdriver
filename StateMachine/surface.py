#!/usr/bin/env python

import rospy
import smach
import smach_ros

# define state Foo
class surface(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Surfaced'])

    def execute(self, userdata):
        rospy.loginfo('Executing state SURFACE')
        return 'Surfaced'