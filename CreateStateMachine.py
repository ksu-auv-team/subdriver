#!/usr/bin/env python

import rospy
import smach
import smach_ros

from StateMachine.exit.exit_track_gate import *
from StateMachine.exit.exit_track_dice import *

from StateMachine.search.search_gate import *
from StateMachine.search.search_dice import *

from StateMachine.track.track_gate import *
from StateMachine.track.track_dice import *

from StateMachine.start import *
from StateMachine.surface import *

def createStateMachine():
    rospy.init_node('AUV_StateMachine')

    # Create the top level SMACH state machine
    sm_AUV = smach.StateMachine(outcomes=['Finished_Run'])
    
    # Open the container
    with sm_AUV:

        smach.StateMachine.add('START', start(), transitions={'Ready_To_Go':'SEARCH_GATE'})

        smach.StateMachine.add('SEARCH_GATE', search_gate(), transitions={'Found_Gate':'TRACK_GATE'})

        smach.StateMachine.add('TRACK_GATE', track_gate(), transitions={'Lost_Gate':'SEARCH_GATE', 'Entered_Gate':'EXIT_TRACK_GATE'})

        smach.StateMachine.add('EXIT_TRACK_GATE', exit_track_gate(), transitions={'Through_Gate':'SEARCH_DICE'})

        smach.StateMachine.add('SEARCH_DICE', search_dice(), transitions={'Found_Dice':'TRACK_DICE'})

        smach.StateMachine.add('TRACK_DICE', track_dice(), transitions={'Lost_Dice':'SEARCH_DICE','Touched_Dice':'EXIT_TRACK_DICE'})

        smach.StateMachine.add('EXIT_TRACK_DICE', exit_track_dice(), transitions={'Clear_Of_Dice':'SURFACE'})

        smach.StateMachine.add('SURFACE', surface(), transitions={'Surfaced':'Finished_Run'})


        # Execute SMACH plan
        outcome = sm_AUV.execute()



def main():
    createStateMachine()


if __name__ == '__main__':
    main()
