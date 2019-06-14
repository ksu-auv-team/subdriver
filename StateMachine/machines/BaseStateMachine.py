#!/usr/bin/env python2

import rospy
import smach

#import literally all the states we will ever have:
from StateMachine.interact.interact_gate import *
from StateMachine.interact.interact_dice import *

from StateMachine.search.search_front import *
from StateMachine.search.search_left import *
from StateMachine.search.search_right import *
from StateMachine.search.search_recenter import *

from StateMachine.search.search_front_gate import *
from StateMachine.search.search_left_gate import *
from StateMachine.search.search_right_gate import *
from StateMachine.search.search_recenter_gate import *

from StateMachine.search.search_front_dice import *
from StateMachine.search.search_left_dice import *
from StateMachine.search.search_right_dice import *
from StateMachine.search.search_recenter_dice import *

from StateMachine.track.track_gate import *
from StateMachine.track.track_dice import *

from StateMachine.taskless.start import *
from StateMachine.taskless.surface import *

def createStateMachine():
    rospy.init_node('AUV_StateMachine')

    # Create the top level SMACH state machine
    sm_AUV = smach.StateMachine(outcomes=['Finished_Run'])
    sm_octagon = smach.StateMachine(outcomes=['Finished_Task', 'Failed_Task'])
    
    # Open the container
    with sm_AUV:

        smach.StateMachine.add('START', start(), transitions={'Not_Found_Gate':'SEARCH_FRONT_GATE', 'Found_Gate':'TRACK_GATE'})

        smach.StateMachine.add('SEARCH_FRONT_GATE', search_front_gate(), transitions={'Found_Object':'TRACK_GATE', 'Not_Found_Object':'SEARCH_LEFT_GATE'})
        smach.StateMachine.add('SEARCH_LEFT_GATE', search_left_gate(), transitions={'Found_Object':'TRACK_GATE', 'Not_Found_Object':'SEARCH_RIGHT_GATE'})
        smach.StateMachine.add('SEARCH_RIGHT_GATE', search_right_gate(), transitions={'Found_Object':'TRACK_GATE', 'Not_Found_Object':'SEARCH_RECENTER_GATE'})
        smach.StateMachine.add('SEARCH_RECENTER_GATE', search_recenter_gate(), transitions={'Found_Object':'TRACK_GATE', 'Not_Found_Object':'SEARCH_FRONT_GATE'})

        smach.StateMachine.add('TRACK_GATE', track_gate(), transitions={'Lost_Gate':'SEARCH_FRONT_GATE', 'Entered_Gate':'INTERACT_GATE'})

        smach.StateMachine.add('INTERACT_GATE', interact_gate(), transitions={'Through_Gate':'SEARCH_FRONT_DICE'})

        smach.StateMachine.add('SEARCH_FRONT_DICE', search_front_dice(), transitions={'Found_Object':'TRACK_DICE', 'Not_Found_Object':'SEARCH_LEFT_DICE'})
        smach.StateMachine.add('SEARCH_LEFT_DICE', search_left_dice(), transitions={'Found_Object':'TRACK_DICE', 'Not_Found_Object':'SEARCH_RIGHT_DICE'})
        smach.StateMachine.add('SEARCH_RIGHT_DICE', search_right_dice(), transitions={'Found_Object':'TRACK_DICE', 'Not_Found_Object':'SEARCH_RECENTER_DICE'})
        smach.StateMachine.add('SEARCH_RECENTER_DICE', search_recenter_dice(), transitions={'Found_Object':'TRACK_DICE', 'Not_Found_Object':'SEARCH_FRONT_DICE'})

        smach.StateMachine.add('TRACK_DICE', track_dice(), transitions={'Lost_Dice':'SEARCH_FRONT_DICE','Touched_Dice':'INTERACT_DICE'})

        smach.StateMachine.add('INTERACT_DICE', interact_dice(), transitions={'Clear_Of_Dice':'SURFACE'})

        smach.StateMachine.add('SEARCH_FRONT_OCTAGON', search_front_dice(), transitions={'Found_Object':'TRACK_OCTAGON', 'Not_Found_Object':'SEARCH_LEFT_OCTAGON'})
        smach.StateMachine.add('SEARCH_LEFT_OCTAGON', search_left_dice(), transitions={'Found_Object':'TRACK_OCTAGON', 'Not_Found_Object':'SEARCH_RIGHT_OCTAGON'})
        smach.StateMachine.add('SEARCH_RIGHT_OCTAGON', search_right_dice(), transitions={'Found_Object':'TRACK_OCTAGON', 'Not_Found_Object':'SEARCH_RECENTER_OCTAGON'})
        smach.StateMachine.add('SEARCH_RECENTER_OCTAGON', search_recenter_dice(), transitions={'Found_Object':'TRACK_OCTAGON', 'Not_Found_Object':'SEARCH_FRONT_OCTAGON'})
        
        smach.StateMachine.add('TRACK_OCTAGON', track_dice(), transitions={'Lost_Object':'SEARCH_FRONT_OCTAGON','Touched_Object':'INTERACT_OCTAGON'})

        #old
        smach.StateMachine.add('SURFACE', surface(), transitions={'Surfaced':'Finished_Run'})

        #octagon interaction sub-state machine
        with sm_octagon:
            smach.StateMachine.add('OCT_START', oct_start(), transitions={'Found_Object':'OCT_CENTER_COFFIN', 'Not_Found_Object':'OCT_FIND_COFFIN'})
            smach.StateMachine.add('OCT_FIND_COFFIN', oct_find_coffin(), transitions={'Found_Object':'OCT_CENTER_COFFIN', 'Not_Found_Object':'Failed_Task'})
            smach.StateMachine.add('OCT_CENTER_COFFIN', oct_center_coffin(), transitions={'Centered_Object':'OCT_RISE'})
            smach.StateMachine.add('OCT_RISE', oct_rise(), transitions={'Surfaced':'OCT_SURFACED'})
            smach.StateMachine.add('OCT_SURFACED', oct_surfaced(), transitions={'Finish_After_Surfacing':'Finished_Task'})
            smach.StateMachine.add('OCT_DIVE')
        smach.StateMachine.add('INTERACT_OCTAGON', sm_octagon, transitions={'Finished_Task':'Finished_Run', 'Failed_Task':'SEARCH_OCTAGON'})




        # Execute SMACH plan
    outcome = sm_AUV.execute()



def main():
    createStateMachine()


if __name__ == '__main__':
    main()
