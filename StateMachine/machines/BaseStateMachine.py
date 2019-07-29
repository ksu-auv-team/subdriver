#!/usr/bin/env python2

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
    sm_AUV = smach.StateMachine(outcomes=['finished_run'])
    sm_octagon = smach.StateMachine(outcomes=['finished_task', 'failed_task'])
    sm_oct_search = smach.StateMachine(outcomes=['search_found'])
    sm_gate_search = smach.StateMachine(outcomes=['search_found'])
    sm_buoy_search = smach.StateMachine(outcomes=['search_found'])
    
    # Open the container
    with sm_AUV:

        #Gate
        smach.StateMachine.add('START', Start(), transitions={'not_found_gate':'SEARCH_GATE', 'found_gate':'TRACK_GATE'})

        with sm_gate_search:
            smach.StateMachine.add('SEARCH_FRONT_GATE', Search_Front_Gate(), transitions={'object_found':'search_found', 'object_not_found':'SEARCH_LEFT_GATE'})
            smach.StateMachine.add('SEARCH_LEFT_GATE', Search_Left_Gate(), transitions={'object_found':'search_found', 'object_not_found':'SEARCH_RIGHT_GATE'})
            smach.StateMachine.add('SEARCH_RIGHT_GATE', Search_Right_Gate(), transitions={'object_found':'search_found', 'object_not_found':'SEARCH_RECENTER_GATE'})
            smach.StateMachine.add('SEARCH_RECENTER_GATE', Search_Recenter_Gate(), transitions={'object_found':'search_found', 'object_not_found':'SEARCH_FRONT_GATE'})

        smach.StateMachine.add('SEARCH_GATE', sm_gate_search, transitions={'search_found':'TRACK_GATE'})
        smach.StateMachine.add('TRACK_GATE', Track_Gate(), transitions={'lost_gate':'SEARCH_GATE', 'approached_gate':'INTERACT_GATE'})
        smach.StateMachine.add('INTERACT_GATE', Interact_Gate(), transitions={'through_gate':'SEARCH_BUOY'})

        #Buoy
        with sm_buoy_search:
            smach.StateMachine.add('SEARCH_FRONT_BUOY', Search_Front_Buoy(), transitions={'object_found':'search_found', 'object_not_found':'SEARCH_LEFT_BUOY'})
            smach.StateMachine.add('SEARCH_LEFT_BUOY', Search_Left_Buoy(), transitions={'object_found':'search_found', 'object_not_found':'SEARCH_RIGHT_BUOY'})
            smach.StateMachine.add('SEARCH_RIGHT_BUOY', Search_Right_Buoy(), transitions={'object_found':'search_found', 'object_not_found':'SEARCH_RECENTER_BUOY'})
            smach.StateMachine.add('SEARCH_RECENTER_BUOY', Search_Recenter_Buoy(), transitions={'object_found':'search_found', 'object_not_found':'SEARCH_FRONT_BUOY'})

        smach.StateMachine.add('SEARCH_BUOY', sm_buoy_search, transitions={'search_found':'TRACK_BUOY'})
        smach.StateMachine.add('TRACK_BUOY', Track_Buoy(), transitions={'lost_buoy':'SEARCH_BUOY','locked_onto_buoy':'INTERACT_BUOY'})
        smach.StateMachine.add('INTERACT_BUOY', Interact_Buoy(), transitions={'clear_of_buoy':'SURFACE'})
        #old
        smach.StateMachine.add('SURFACE', Surface(), transitions={'surfaced':'finished_run'})


        #TODO: Finish octagon states and add to state machine
        # with sm_oct_search:
        #     smach.StateMachine.add('SEARCH_FRONT_OCTAGON', Search_Front_Octagon), transitions={'object_found':'TRACK_OCTAGON', 'object_not_found':'SEARCH_LEFT_OCTAGON'})
        #     smach.StateMachine.add('SEARCH_LEFT_OCTAGON', Search_Left_Octagon(), transitions={'object_found':'TRACK_OCTAGON', 'object_not_found':'SEARCH_RIGHT_OCTAGON'})
        #     smach.StateMachine.add('SEARCH_RIGHT_OCTAGON', Search_Right_Octagon(), transitions={'object_found':'TRACK_OCTAGON', 'object_not_found':'SEARCH_RECENTER_OCTAGON'})
        #     smach.StateMachine.add('SEARCH_RECENTER_OCTAGON', Search_Recenter_Octagon(), transitions={'object_found':'TRACK_OCTAGON', 'object_not_found':'SEARCH_FRONT_OCTAGON'})
        # smach.StateMachine.add('SEARCH_OCTAGON', sm_oct_search, transitions={'search_found':'TRACK_OCTAGON'}

        # smach.StateMachine.add('TRACK_OCTAGON', track_octagon), transitions={'lost_object':'SEARCH_FRONT_OCTAGON','reached_object':'INTERACT_OCTAGON'})


        # #octagon interaction sub-state machine
        # with sm_octagon:
        #     smach.StateMachine.add('OCT_START', oct_start(), transitions={'object_found':'OCT_CENTER_COFFIN', 'object_not_found':'OCT_FIND_COFFIN'})
        #     smach.StateMachine.add('OCT_FIND_COFFIN', oct_find_coffin(), transitions={'object_found':'OCT_CENTER_COFFIN', 'object_not_found':'Failed_Task'})
        #     smach.StateMachine.add('OCT_CENTER_COFFIN', oct_center_coffin(), transitions={'centered_object':'OCT_RISE'})
        #     smach.StateMachine.add('OCT_RISE', oct_rise(), transitions={'surfaced':'OCT_SURFACED'})
        #     smach.StateMachine.add('OCT_SURFACED', oct_surfaced(), transitions={'finish_after_surfacing':'Finished_Task'})
        #     smach.StateMachine.add('OCT_DIVE')
        # smach.StateMachine.add('INTERACT_OCTAGON', sm_octagon, transitions={'finished_task':'finished_run', 'failed_task':'SEARCH_OCTAGON'})

        #TODO(travis): Integrate the torpedo actions in appropriate transition points (depends upon target acquition and tracking states).
        #smach.StateMachine.add('INTERACT_TORPEDO', interact_torpedo(), transitions={'torpedo_launched':'SUCCESS_STATE','torpedo_failed':'FAILURE_STATE'})

        #Octagon

    # Execute SMACH plan
    outcome = sm_AUV.execute()



def main():
    createStateMachine()


if __name__ == '__main__':
    main()
