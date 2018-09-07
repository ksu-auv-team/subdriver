#!/usr/bin/env python

import rospy
import smach
import smach_ros

from StateMachine.sub import *

# define state Foo
class track_dice(sub):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Lost_Dice','Touched_Dice'])

    def execute(self, userdata):
        rospy.loginfo('Executing state TRACK_DICE')
        return 'Touched_Dice'