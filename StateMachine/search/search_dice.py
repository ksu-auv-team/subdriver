#!/usr/bin/env python

import rospy
import smach
import smach_ros

# define state Foo
class search_dice(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Found_Dice'])

    def execute(self, userdata):
        rospy.loginfo('Executing state SEARCH_DICE')
        return 'Found_Dice'