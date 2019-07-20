#!/usr/bin/env python

import rospy
import smach

#import literally all the states we will ever have:
from StateMachine.interact.interact_gate import *
from StateMachine.interact.interact_buoy import *

from StateMachine.search.search_front import *
from StateMachine.search.search_left import *
from StateMachine.search.search_right import *
from StateMachine.search.search_recenter import *

from StateMachine.search.search_front_gate import *
from StateMachine.search.search_left_gate import *
from StateMachine.search.search_right_gate import *
from StateMachine.search.search_recenter_gate import *

from StateMachine.search.search_front_buoy import *
from StateMachine.search.search_left_buoy import *
from StateMachine.search.search_right_buoy import *
from StateMachine.search.search_recenter_buoy import *

from StateMachine.track.track_gate import *
from StateMachine.track.track_buoy import *

from StateMachine.taskless.start import *
from StateMachine.taskless.surface import *

def createStateMachine():
    rospy.init_node('AUV_StateMachine')

    # Create the top level SMACH state machine
    sm_AUV = smach.StateMachine(outcomes=['Finished_Run'])
    
    # Open the container
    with sm_AUV:

        smach.StateMachine.add('START', start(), transitions={'Not_Found_Gate':'SEARCH_FRONT_GATE', 'Found_Gate':'TRACK_GATE'})

        smach.StateMachine.add('SEARCH_FRONT_GATE', search_front_gate(), transitions={'Found_Object':'TRACK_GATE', 'Not_Found_Object':'SEARCH_LEFT_GATE'})
        smach.StateMachine.add('SEARCH_LEFT_GATE', search_left_gate(), transitions={'Found_Object':'TRACK_GATE', 'Not_Found_Object':'SEARCH_RIGHT_GATE'})
        smach.StateMachine.add('SEARCH_RIGHT_GATE', search_right_gate(), transitions={'Found_Object':'TRACK_GATE', 'Not_Found_Object':'SEARCH_RECENTER_GATE'})
        smach.StateMachine.add('SEARCH_RECENTER_GATE', search_recenter_gate(), transitions={'Found_Object':'TRACK_GATE', 'Not_Found_Object':'SEARCH_FRONT_GATE'})

        smach.StateMachine.add('TRACK_GATE', track_gate(), transitions={'Lost_Gate':'SEARCH_FRONT_GATE', 'Approached_Gate':'INTERACT_GATE'})

        smach.StateMachine.add('INTERACT_GATE', interact_gate(), transitions={'Through_Gate':'SEARCH_FRONT_BUOY'})

        smach.StateMachine.add('SEARCH_FRONT_BUOY', search_front_buoy(), transitions={'Found_Object':'TRACK_BUOY', 'Not_Found_Object':'SEARCH_LEFT_BUOY'})
        smach.StateMachine.add('SEARCH_LEFT_BUOY', search_left_buoy(), transitions={'Found_Object':'TRACK_BUOY', 'Not_Found_Object':'SEARCH_RIGHT_BUOY'})
        smach.StateMachine.add('SEARCH_RIGHT_BUOY', search_right_buoy(), transitions={'Found_Object':'TRACK_BUOY', 'Not_Found_Object':'SEARCH_RECENTER_BUOY'})
        smach.StateMachine.add('SEARCH_RECENTER_BUOY', search_recenter_buoy(), transitions={'Found_Object':'TRACK_BUOY', 'Not_Found_Object':'SEARCH_FRONT_BUOY'})

        smach.StateMachine.add('TRACK_BUOY', track_buoy(), transitions={'Lost_buoy':'SEARCH_FRONT_BUOY','Locked_Onto_buoy':'INTERACT_BUOY'})

        smach.StateMachine.add('INTERACT_BUOY', interact_buoy(), transitions={'Clear_Of_Buoy':'SURFACE'})

        smach.StateMachine.add('SURFACE', surface(), transitions={'Surfaced':'Finished_Run'})


        # Execute SMACH plan
    outcome = sm_AUV.execute()



def main():
    createStateMachine()


if __name__ == '__main__':
    main()
