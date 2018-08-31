#!/usr/bin/env python

import rospy
import smach
import smach_ros

# define state Foo
class interact_gate(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Through_Gate'])

    def execute(self, userdata):
        rospy.loginfo('Executing state INTERACT_GATE')
        return 'Through_Gate'