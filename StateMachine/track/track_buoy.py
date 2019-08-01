#!/usr/bin/env python

from StateMachine.sub import *
from StateMachine import controllers

#TODO: clean up - this needs more work than I'm willing to do right now (while merging stuff)
# add a buoy class group, and reference PID() and msg correctly

# define state track_buoy
class Track_Buoy(Sub):
    """ This state starts with the sub having the buoy in sight.
    In this state, the sub will adjust its depth to match the 3 sided buoy and angle to face it. 
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['lost_buoy','locked_onto_buoy'])

    def execute(self, userdata):
        rospy.loginfo('Executing state TRACK_BUOY')
        self.init_state()

        # Start the front network
        self.use_front_network(True)

        msg = self.init_joy_msg()

        if (self.get_box_of_class(gbl.detections_front, const.CLASSES['buoy_aswang']) is None \
            and self.get_box_of_class(gbl.detections_front, const.CLASSES['buoy_aswang']) is None \
            and self.get_box_of_class(gbl.detections_front, const.CLASSES['buoy_aswang']) is None \
            and self.get_box_of_class(gbl.detections_front, const.CLASSES['buoy_aswang'])) is None \
            and (rospy.get_time() - self.current_state_start_time) > 2:
            
            rospy.logwarn("Lost tracking for more than 2 seconds")
            return "lost_buoy"

        while self.get_center(self.get_boxes_of_classes(gbl.detections_front, const.BUOY_FACES)) != 0:
            pass

        # At this point, the sub is stationary and facing the Buoy
        return 'locked_onto_buoy'


        