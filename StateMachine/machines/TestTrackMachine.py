#!/usr/bin/env python2

import rospy
import smach

from StateMachine.taskless.start import *
from StateMachine.taskless.surface import *

from StateMachine.track.track_gate import *

def createStateMachine():
    rospy.init_node('AUV_StateMachine')

    # Create the top level SMACH state machine
    sm_AUV = smach.StateMachine(outcomes=['finished_run'])
    
    # Open the container
    with sm_AUV:

        smach.StateMachine.add('START', Start(), transitions={'not_found_gate':'TRACK_GATE', 'found_gate':'TRACK_GATE'})

        smach.StateMachine.add('TRACK_GATE', Track_Gate(), transitions={'lost_gate':'TRACK_GATE', 'approached_gate':'TRACK_GATE'})

        smach.StateMachine.add('SURFACE', Surface(), transitions={'surfaced':'finished_run'})


    # Execute SMACH plan
    outcome = sm_AUV.execute()



def main():
    createStateMachine()


if __name__ == '__main__':
    main()
