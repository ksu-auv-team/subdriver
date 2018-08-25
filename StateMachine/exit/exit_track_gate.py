#!/usr/bin/env python

import rospy
import smach
import smach_ros

# define state Foo
class exit_track_gate(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Through_Gate'])

    def execute(self, userdata):
        rospy.loginfo('Executing state EXIT_TRACK_GATE')
        return 'Through_Gate'