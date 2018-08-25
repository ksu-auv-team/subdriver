#!/usr/bin/env python

import rospy
import smach
import smach_ros

# define state Foo
class track_gate(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Lost_Gate','Entered_Gate'])

    def execute(self, userdata):
        rospy.loginfo('Executing state TRACK_GATE')
        return 'Entered_Gate'