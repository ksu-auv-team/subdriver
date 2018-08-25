#!/usr/bin/env python

import rospy
import smach
import smach_ros


# define state Foo
class start(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Ready_To_Go'])

    def execute(self, userdata):
        rospy.loginfo('Executing state START')
        return 'Ready_To_Go'