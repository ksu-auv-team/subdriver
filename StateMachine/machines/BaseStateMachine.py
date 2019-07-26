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
    sm_AUV = smach.StateMachine(outcomes=['Finished_Run'])
    sm_octagon = smach.StateMachine(outcomes=['Finished_Task', 'Failed_Task'])
    sm_oct_search = smach.StateMachine(outcomes=['Found_Object'])
    
    # Open the container
    with sm_AUV:

        smach.StateMachine.add('START', Start(), transitions={'Not_Found_Gate':'SEARCH_FRONT_GATE', 'Found_Gate':'TRACK_GATE'})

        smach.StateMachine.add('SEARCH_FRONT_GATE', Search_Front_Gate(), transitions={'Found_Object':'TRACK_GATE', 'Not_Found_Object':'SEARCH_LEFT_GATE'})
        smach.StateMachine.add('SEARCH_LEFT_GATE', Search_Left_Gate(), transitions={'Found_Object':'TRACK_GATE', 'Not_Found_Object':'SEARCH_RIGHT_GATE'})
        smach.StateMachine.add('SEARCH_RIGHT_GATE', Search_Right_Gate(), transitions={'Found_Object':'TRACK_GATE', 'Not_Found_Object':'SEARCH_RECENTER_GATE'})
        smach.StateMachine.add('SEARCH_RECENTER_GATE', Search_Recenter_Gate(), transitions={'Found_Object':'TRACK_GATE', 'Not_Found_Object':'SEARCH_FRONT_GATE'})

        smach.StateMachine.add('TRACK_GATE', Track_Gate(), transitions={'Lost_Gate':'SEARCH_FRONT_GATE', 'Approached_Gate':'INTERACT_GATE'})

        smach.StateMachine.add('INTERACT_GATE', Interact_Gate(), transitions={'Through_Gate':'SEARCH_FRONT_BUOY'})

        smach.StateMachine.add('SEARCH_FRONT_BUOY', Search_Front_Buoy(), transitions={'Found_Object':'TRACK_BUOY', 'Not_Found_Object':'SEARCH_LEFT_BUOY'})
        smach.StateMachine.add('SEARCH_LEFT_BUOY', Search_Left_Buoy(), transitions={'Found_Object':'TRACK_BUOY', 'Not_Found_Object':'SEARCH_RIGHT_BUOY'})
        smach.StateMachine.add('SEARCH_RIGHT_BUOY', Search_Right_Buoy(), transitions={'Found_Object':'TRACK_BUOY', 'Not_Found_Object':'SEARCH_RECENTER_BUOY'})
        smach.StateMachine.add('SEARCH_RECENTER_BUOY', Search_Recenter_Buoy(), transitions={'Found_Object':'TRACK_BUOY', 'Not_Found_Object':'SEARCH_FRONT_BUOY'})

        smach.StateMachine.add('TRACK_BUOY', Track_Buoy(), transitions={'Lost_Buoy':'SEARCH_FRONT_BUOY','Locked_Onto_Buoy':'INTERACT_BUOY'})

        smach.StateMachine.add('INTERACT_BUOY', Interact_Buoy(), transitions={'Clear_Of_Buoy':'SURFACE'})
        #old
        smach.StateMachine.add('SURFACE', Surface(), transitions={'Surfaced':'Finished_Run'})


        #TODO: Finish octagon states and add to state machine
        # with sm_oct_search:
        #     smach.StateMachine.add('SEARCH_FRONT_OCTAGON', Search_Front_Octagon), transitions={'Found_Object':'TRACK_OCTAGON', 'Not_Found_Object':'SEARCH_LEFT_OCTAGON'})
        #     smach.StateMachine.add('SEARCH_LEFT_OCTAGON', Search_Left_Octagon(), transitions={'Found_Object':'TRACK_OCTAGON', 'Not_Found_Object':'SEARCH_RIGHT_OCTAGON'})
        #     smach.StateMachine.add('SEARCH_RIGHT_OCTAGON', Search_Right_Octagon(), transitions={'Found_Object':'TRACK_OCTAGON', 'Not_Found_Object':'SEARCH_RECENTER_OCTAGON'})
        #     smach.StateMachine.add('SEARCH_RECENTER_OCTAGON', Search_Recenter_Octagon(), transitions={'Found_Object':'TRACK_OCTAGON', 'Not_Found_Object':'SEARCH_FRONT_OCTAGON'})
        # smach.StateMachine.add('SEARCH_OCTAGON', sm_oct_search, transitions={'Found_Object':''}

        # smach.StateMachine.add('TRACK_OCTAGON', track_dice(), transitions={'Lost_Object':'SEARCH_FRONT_OCTAGON','Touched_Object':'INTERACT_OCTAGON'})


        # #octagon interaction sub-state machine
        # with sm_octagon:
        #     smach.StateMachine.add('OCT_START', oct_start(), transitions={'Found_Object':'OCT_CENTER_COFFIN', 'Not_Found_Object':'OCT_FIND_COFFIN'})
        #     smach.StateMachine.add('OCT_FIND_COFFIN', oct_find_coffin(), transitions={'Found_Object':'OCT_CENTER_COFFIN', 'Not_Found_Object':'Failed_Task'})
        #     smach.StateMachine.add('OCT_CENTER_COFFIN', oct_center_coffin(), transitions={'Centered_Object':'OCT_RISE'})
        #     smach.StateMachine.add('OCT_RISE', oct_rise(), transitions={'Surfaced':'OCT_SURFACED'})
        #     smach.StateMachine.add('OCT_SURFACED', oct_surfaced(), transitions={'Finish_After_Surfacing':'Finished_Task'})
        #     smach.StateMachine.add('OCT_DIVE')
        # smach.StateMachine.add('INTERACT_OCTAGON', sm_octagon, transitions={'Finished_Task':'Finished_Run', 'Failed_Task':'SEARCH_OCTAGON'})

        #TODO(travis): Integrate the torpedo actions in appropriate transition points (depends upon target acquition and tracking states).
        #smach.StateMachine.add('INTERACT_TORPEDO', interact_torpedo(), transitions={'Torpedo_Launched':'SUCCESS_STATE','Torpedo_Failed':'FAILURE_STATE'})

        # Execute SMACH plan
    outcome = sm_AUV.execute()



def main():
    createStateMachine()


if __name__ == '__main__':
    main()
