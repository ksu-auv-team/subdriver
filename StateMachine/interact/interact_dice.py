#!/usr/bin/env python

import rospy
import smach
import smach_ros

# define state Foo
class interact_dice(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Clear_Of_Dice'])

    def execute(self, userdata):
        rospy.loginfo('Executing state INTERACT_DICE')
        return 'Clear_Of_Dice'