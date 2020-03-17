#!/usr/bin/env python2

import rospy
import smach

from StateMachine.taskless.dumb_start import *
from StateMachine.taskless.surface import *

from StateMachine.taskless.getLow import *

def createStateMachine():
    rospy.init_node('AUV_StateMachine')

    # Create the top level SMACH state machine
    sm_AUV = smach.StateMachine(outcomes=['finished_run'])
    
    # Open the container
    with sm_AUV:

        smach.StateMachine.add('DUMB_START', Dumb_Start(), transitions={'setup_complete':'DIVE'})

        smach.StateMachine.add('DIVE', Dive(), transitions={'dived':'SURFACE'})

        smach.StateMachine.add('SURFACE', Surface(), transitions={'surfaced':'finished_run'})


    # Execute SMACH plan
    outcome = sm_AUV.execute()



def main():
    createStateMachine()


if __name__ == '__main__':
    main()
