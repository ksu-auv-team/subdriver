#!/usr/bin/env python

from StateMachine.sub import *

# define state interact_dice
class interact_dice(sub):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Clear_Of_Dice'])

    def execute(self, userdata):
        rospy.loginfo('Executing state INTERACT_DICE')
        return 'Clear_Of_Dice'