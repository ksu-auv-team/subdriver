#!/usr/bin/env python
from StateMachine.sub import *

'''
Defines substate of Sub that will center us on a buoy.
It's designed to use current_target so we can use it on multiple faces.
Expects to start near enough to the buoys that we can see them, but not, like, super close?
Ideally we'd be able to see both.

Will get the sub to the center of the face first
'''


class Center_On_Buoy(Sub):
    def __init__(self):
        smach.State.__init__(self, outcomes=['centered_first_buoy', 'centered_second_buoy', 'lost_buoy'])

    def execute(self, userdata):
        rospy.loginfo('Executing state CENTER_ON_BUOY')
        self.init_state()
        msg = self.init_joy_msg()  

        # We should have started seeing this
        self.last_seen = rospy.get_time()

        # Initalize front network
        self.use_front_network(True)   

        while True:
            curr_det = self.get_box_of_class(gbl.detections_front, gbl.current_target)

            # If the detection is good, center on it
            if curr_det:
                msg = self.align_with_screen(curr_det.box)
                self.last_seen = rospy.get_time()

                if(rospy.get_time() - self.current_state_start_time) > 1 and gbl.buoy_num is 1:
                    gbl.buoy_num = 2
                    return 'centered_first_buoy'

                elif (rospy.get_time() - self.current_state_start_time) > 1 and gbl.buoy_num is 2:
                    return 'centered_second_buoy'

            # If the detection is bad, stop moving
            else:
                msg = self.init_joy_msg()

            # If the detection is bad for a while, search
            if (rospy.get_time() - self.last_seen) > 4:
                rospy.logwarn("Lost tracking for more than 4 seconds")
                
                if gbl.debug:
                    if gbl.buoy_num is 1:
                        gbl.buoy_num = 2
                        return 'centered_first_buoy'
                    if gbl.buoy_num is 2:
                        return 'centered_second_buoy'
                        
                return 'lost_buoy'

            self.publish_joy(msg)
            rospy.sleep(const.SLEEP_TIME)
