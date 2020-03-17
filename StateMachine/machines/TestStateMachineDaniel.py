#!/usr/bin/env python2

import rospy
import smach

from StateMachine.test_states.test_start import *
from StateMachine.test_states.rotate_left import *
from StateMachine.test_states.rotate_right import *
from StateMachine.taskless.surface import *


def createStateMachine():
    rospy.init_node('AUV_StateMachine')

    # Create the top level SMACH state machine
    sm_AUV = smach.StateMachine(outcomes=['finished_run'])

    # Open the container
    with sm_AUV:
        smach.StateMachine.add('TEST_START', Test_Start(), transitions={'rotate_left': 'ROTATE_LEFT'})

        smach.StateMachine.add('ROTATE_LEFT', Rotate_Left(), transitions={'surface': 'SURFACE'})

        smach.StateMachine.add('SURFACE', Surface(), transitions={'surfaced': 'finished_run'})

    # Execute SMACH plan
    outcome = sm_AUV.execute()


def main():
    createStateMachine()

if __name__ == '__main__':
    main()