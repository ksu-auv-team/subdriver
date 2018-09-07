#!/usr/bin/env python

import rospy
import smach
import smach_ros


# define state Foo
class sub(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Finished_Run'])

    def execute(self, userdata):
        rospy.loginfo('Initializing State Machine...')
        return 'Finished_Run'

    def log(self):
    	pass

    def depth_hold(self):
    	pass