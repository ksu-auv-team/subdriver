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
        smach.State.__init__(self, outcomes=['lost_buoy','approached_buoy'])

    def execute(self, userdata):
        rospy.loginfo('Executing state TRACK_BUOY')
        self.init_state()
        self.last_seen = rospy.get_time()
        gbl.state_heading = gbl.heading

        # Start the front network
        self.use_front_network(True)

        msg = self.init_joy_msg()

        if self.get_box_of_class(gbl.detections_front, gbl.current_target) is None and (rospy.get_time() - self.current_state_start_time) > 2:
            rospy.logwarn("Lost tracking for more than 2 seconds")
            return "lost_buoy"

        #control loop
        while(1):
            msg = self.init_joy_msg()
            detection = self.get_box_of_class(gbl.detections_front, gbl.current_target)

            if (detection != None) and detection.score > 0.3:  # If the box is good
                self.last_seen = rospy.get_time()
                center = self.get_center(detection.box)

                #### Use this instead of align_with_box if it doesn't work ####

                # if center[0] < 0.45:
                #     msg.axes[const.AXES['rotate']] = 0.2
                # elif center[0] > 0.55:
                #     msg.axes[const.AXES['rotate']] = -0.2

                # if center[1] < 0.45:
                #     msg.axes[const.AXES['vertical']] = 0.2
                # elif center[1] > 0.55:
                #     msg.axes[const.AXES['vertical']] = -0.2

                msg = self.align_with_box(detection.box)                
                msg.axes[const.AXES['frontback']] = 0.3
                
                if detection:
                    if self.get_distance(detection.box[0], detection.box[1], detection.box[2], detection.box[3]) > 0.9 or detection.box[3] - detection.box[1] > 0.8:
                        self.is_close = True

                #stay within 35 degrees of state initial heading
                #we'll always start pointed at the gate, so we'll never want more than that unless something goes wrong.
                if self.angle_diff(gbl.heading, gbl.state_heading) > 35:
                    #go left
                    msg.axes[const.AXES['rotate']] = 0.2
                elif self.angle_diff(gbl.heading, gbl.state_heading) < -35:
                    #go right
                    msg.axes[const.AXES['rotate']] = -0.2


            if self.is_close:
                self.is_close = False
                return "approached_buoy" # Transitions to INTERACT_GATE
            elif (rospy.get_time() - self.last_seen) > 2:
                msg.axes[const.AXES['frontback']] = 0
                self.publish_joy(msg)
                rospy.logwarn("Lost tracking the buoys for more than 2 seconds")
                
                if(gbl.debug):
                    return "approached_buoy" # DEBUG Porpoises Only!
                
                return "lost_buoy" # Transitions to SEARCH_FRONT_GATE

            self.publish_joy(msg)

            rospy.sleep(const.SLEEP_TIME)

        