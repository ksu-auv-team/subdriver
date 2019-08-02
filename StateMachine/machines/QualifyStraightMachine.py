#!/usr/bin/env python

import rospy
import smach

#import literally all the states we will ever have:
from StateMachine.search.search_front import *
from StateMachine.search.search_left import *
from StateMachine.search.search_right import *
from StateMachine.search.search_recenter import *

from StateMachine.search.search_front_buoy import *
from StateMachine.search.search_left_buoy import *
from StateMachine.search.search_right_buoy import *
from StateMachine.search.search_recenter_buoy import *

from StateMachine.track.track_buoy import *

from StateMachine.interact.buoy.bump_buoy import *
from StateMachine.interact.buoy.center_on_buoy import *
from StateMachine.interact.buoy.shift_buoy import *

from StateMachine.taskless.start import *
from StateMachine.taskless.surface import *
from StateMachine.taskless.straight_ahead import *
from StateMachine.taskless.spin2win import *

def createStateMachine():
    rospy.init_node('AUV_StateMachine')

    # Create the top level SMACH state machine
    sm_AUV = smach.StateMachine(outcomes=['Finished_Run'])
    sm_buoy_search = smach.StateMachine(outcomes=['search_found'])
    sm_buoy_interact = smach.StateMachine(outcomes=['lost_buoy', 'clear_of_buoys'])

    # Open the container
    with sm_AUV:

        smach.StateMachine.add('START', Start(), transitions={'not_found_gate':'STRAIGHT_AHEAD', 'found_gate':'STRAIGHT_AHEAD'})

        smach.StateMachine.add('STRAIGHT_AHEAD', Straight_Ahead(), transitions={'through_gate':'SPIN_2_WIN'})
        
        smach.StateMachine.add('SPIN_2_WIN', SpinToWin(), transitions={'through_gate':'SEARCH_BUOY', 'found_buoy':'TRACK_BUOY'})

        with sm_buoy_search:
            smach.StateMachine.add('SEARCH_FRONT_BUOY', Search_Front_Buoy(), transitions={'object_found':'search_found', 'object_not_found':'SEARCH_LEFT_BUOY'})
            smach.StateMachine.add('SEARCH_LEFT_BUOY', Search_Left_Buoy(), transitions={'object_found':'search_found', 'object_not_found':'SEARCH_RIGHT_BUOY'})
            smach.StateMachine.add('SEARCH_RIGHT_BUOY', Search_Right_Buoy(), transitions={'object_found':'search_found', 'object_not_found':'SEARCH_RECENTER_BUOY'})
            smach.StateMachine.add('SEARCH_RECENTER_BUOY', Search_Recenter_Buoy(), transitions={'object_found':'search_found', 'object_not_found':'SEARCH_FRONT_BUOY'})

        smach.StateMachine.add('SEARCH_BUOY', sm_buoy_search, transitions={'search_found':'TRACK_BUOY'})
        smach.StateMachine.add('TRACK_BUOY', Track_Buoy(), transitions={'lost_buoy':'SEARCH_BUOY','approached_buoy':'INTERACT_BUOY'})
        #smach.StateMachine.add('BUMP_BUOY', Bump_Buoy(), transitions={'bumped_first_buoy':'SURFACE', 'bumped_second_buoy':'SURFACE'})

        with sm_buoy_interact:
            smach.StateMachine.add('INTERACT_BUOY_CENTER', Center_On_Buoy(), transitions={'centered_first_buoy':'INTERACT_BUOY_BUMP', 'centered_second_buoy':'INTERACT_BUOY_BUMP', 'lost_buoy':'lost_buoy'})            
            smach.StateMachine.add('INTERACT_BUOY_BUMP', Bump_Buoy(), transitions={'bumped_first_buoy':'INTERACT_BUOY_SHIFT', 'bumped_second_buoy':'clear_of_buoys'})
            smach.StateMachine.add('INTERACT_BUOY_SHIFT', Shift_Buoy(), transitions={'finished_shifting':'INTERACT_BUOY_BUMP'})                        

        smach.StateMachine.add('INTERACT_BUOY', sm_buoy_interact, transitions={'clear_of_buoys':'SURFACE','lost_buoy':'SEARCH_BUOY'})        

        smach.StateMachine.add('SURFACE', Surface(), transitions={'surfaced':'Finished_Run'})

        # Execute SMACH plan
    outcome = sm_AUV.execute()



def main():
    createStateMachine()


if __name__ == '__main__':
    main()
