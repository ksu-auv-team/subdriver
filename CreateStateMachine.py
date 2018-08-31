#!/usr/bin/env python

import rospy
import smach
import smach_ros

from StateMachine.exit.exit_track_gate import *
from StateMachine.exit.exit_track_dice import *

from StateMachine.search.search_gate import *
from StateMachine.search.search_left_gate import *
from StateMachine.search.search_right_gate import *
from StateMachine.search.search_back_gate import *
from StateMachine.search.search_front_gate import *

from StateMachine.search.search_dice import *
from StateMachine.search.search_left_dice import *
from StateMachine.search.search_right_dice import *
from StateMachine.search.search_back_dice import *
from StateMachine.search.search_front_dice import *

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

        smach.StateMachine.add('START', start(), transitions={'Ready_To_Go':'SEARCH_LEFT_GATE'})

        smach.StateMachine.add('SEARCH_LEFT_GATE', search_left_gate(), transitions={'Found_Object':'TRACK_GATE', 'Not_Found_Object':'SEARCH_RIGHT_GATE'})
        smach.StateMachine.add('SEARCH_RIGHT_GATE', search_right_gate(), transitions={'Found_Object':'TRACK_GATE', 'Not_Found_Object':'SEARCH_BACK_GATE'})
        smach.StateMachine.add('SEARCH_BACK_GATE', search_back_gate(), transitions={'Found_Object':'TRACK_GATE', 'Not_Found_Object':'SEARCH_FRONT_GATE'})
        smach.StateMachine.add('SEARCH_FRONT_GATE', search_front_gate(), transitions={'Found_Object':'TRACK_GATE', 'Not_Found_Object':'SEARCH_LEFT_GATE'})

        smach.StateMachine.add('TRACK_GATE', track_gate(), transitions={'Lost_Gate':'SEARCH_LEFT_GATE', 'Entered_Gate':'EXIT_TRACK_GATE'})

        smach.StateMachine.add('EXIT_TRACK_GATE', exit_track_gate(), transitions={'Through_Gate':'SEARCH_LEFT_DICE'})

        smach.StateMachine.add('SEARCH_LEFT_DICE', search_left_dice(), transitions={'Found_Object':'TRACK_DICE', 'Not_Found_Object':'SEARCH_RIGHT_DICE'})
        smach.StateMachine.add('SEARCH_RIGHT_DICE', search_right_dice(), transitions={'Found_Object':'TRACK_DICE', 'Not_Found_Object':'SEARCH_BACK_DICE'})
        smach.StateMachine.add('SEARCH_BACK_DICE', search_back_dice(), transitions={'Found_Object':'TRACK_DICE', 'Not_Found_Object':'SEARCH_FRONT_DICE'})
        smach.StateMachine.add('SEARCH_FRONT_DICE', search_front_dice(), transitions={'Found_Object':'TRACK_DICE', 'Not_Found_Object':'SEARCH_LEFT_DICE'})

        smach.StateMachine.add('TRACK_DICE', track_dice(), transitions={'Lost_Dice':'SEARCH_LEFT_DICE','Touched_Dice':'EXIT_TRACK_DICE'})

        smach.StateMachine.add('EXIT_TRACK_DICE', exit_track_dice(), transitions={'Clear_Of_Dice':'SURFACE'})

        smach.StateMachine.add('SURFACE', surface(), transitions={'Surfaced':'Finished_Run'})


        # Execute SMACH plan
        outcome = sm_AUV.execute()



def main():
    createStateMachine()


if __name__ == '__main__':
    main()
