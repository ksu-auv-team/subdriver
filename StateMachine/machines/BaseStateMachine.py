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

        sm_gate_search = smach.StateMachine(outcomes=['search_found'])
        sm_buoy_search = smach.StateMachine(outcomes=['search_found'])

        #Gate
        smach.StateMachine.add('START', Start(), transitions={'Not_Found_Gate':'SEARCH_FRONT_GATE', 'Found_Gate':'TRACK_GATE'})

        with sm_gate_search:
            smach.StateMachine.add('SEARCH_FRONT_GATE', Search_Front_Gate(), transitions={'Found_Object':'search_found', 'Not_Found_Object':'SEARCH_LEFT_GATE'})
            smach.StateMachine.add('SEARCH_LEFT_GATE', Search_Left_Gate(), transitions={'Found_Object':'search_found', 'Not_Found_Object':'SEARCH_RIGHT_GATE'})
            smach.StateMachine.add('SEARCH_RIGHT_GATE', Search_Right_Gate(), transitions={'Found_Object':'search_found', 'Not_Found_Object':'SEARCH_RECENTER_GATE'})
            smach.StateMachine.add('SEARCH_RECENTER_GATE', Search_Recenter_Gate(), transitions={'Found_Object':'search_found', 'Not_Found_Object':'SEARCH_FRONT_GATE'})

        smach.StateMachine.add('SEARCH_GATE', sm_gate_search, transitions={'search_found':'TRACK_GATE'})
        smach.StateMachine.add('TRACK_GATE', Track_Gate(), transitions={'Lost_Gate':'SEARCH_FRONT_GATE', 'Approached_Gate':'INTERACT_GATE'})
        smach.StateMachine.add('INTERACT_GATE', Interact_Gate(), transitions={'Through_Gate':'SEARCH_FRONT_BUOY'})

        #Buoy
        with sm_buoy_search:
            smach.StateMachine.add('SEARCH_FRONT_BUOY', Search_Front_Buoy(), transitions={'Found_Object':'TRACK_BUOY', 'Not_Found_Object':'SEARCH_LEFT_BUOY'})
            smach.StateMachine.add('SEARCH_LEFT_BUOY', Search_Left_Buoy(), transitions={'Found_Object':'TRACK_BUOY', 'Not_Found_Object':'SEARCH_RIGHT_BUOY'})
            smach.StateMachine.add('SEARCH_RIGHT_BUOY', Search_Right_Buoy(), transitions={'Found_Object':'TRACK_BUOY', 'Not_Found_Object':'SEARCH_RECENTER_BUOY'})
            smach.StateMachine.add('SEARCH_RECENTER_BUOY', Search_Recenter_Buoy(), transitions={'Found_Object':'TRACK_BUOY', 'Not_Found_Object':'SEARCH_FRONT_BUOY'})

        smach.StateMachine.add('SEARCH_BUOY', sm_buoy_search, transitions={'search_found':'TRACK_BUOY'})
        smach.StateMachine.add('TRACK_BUOY', Track_Buoy(), transitions={'Lost_Buoy':'SEARCH_FRONT_BUOY','Locked_Onto_Buoy':'INTERACT_BUOY'})
        smach.StateMachine.add('INTERACT_BUOY', Interact_Buoy(), transitions={'Clear_Of_Buoy':'SURFACE'})

        #Torpedo

        smach.StateMachine.add('SURFACE', Surface(), transitions={'Surfaced':'Finished_Run'})

        #TODO(travis): Integrate the torpedo actions in appropriate transition points (depends upon target acquition and tracking states).
        #smach.StateMachine.add('INTERACT_TORPEDO', interact_torpedo(), transitions={'Torpedo_Launched':'SUCCESS_STATE','Torpedo_Failed':'FAILURE_STATE'})

        #Octagon

    # Execute SMACH plan
    outcome = sm_AUV.execute()



def main():
    createStateMachine()


if __name__ == '__main__':
    main()
