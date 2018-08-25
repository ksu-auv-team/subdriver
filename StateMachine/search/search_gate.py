#!/usr/bin/env python

import rospy
import smach
import smach_ros

# define state Foo
class search_gate(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Found_Gate'])

    def execute(self, userdata):
        rospy.loginfo('Executing state SEARCH_GATE')
        return 'Found_Gate'