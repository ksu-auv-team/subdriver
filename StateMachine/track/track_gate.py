#!/usr/bin/env python

from StateMachine.sub import *

# define state track_gate
class track_gate(sub):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Lost_Gate','Approached_Gate'])

    def execute(self, userdata):
        self.init_state()
        self.last_seen = rospy.get_time()

        #control loop
        while(1):
            msg = self.init_joy_msg()
            detection = self.get_box_of_class(gbl.detections, gbl.current_target)

            if (detection != None) and detection.score > 0.3:  # If the box is good
                self.last_seen = rospy.get_time()
                center = self.getCenter(detection.box)
                msg.axes[const.AXES['frontback']] = 0.3
  
                if center[0] < 0.45:
                    msg.axes[const.AXES['rotate']] = 0.05
                elif center[0] > 0.55:
                    msg.axes[const.AXES['rotate']] = -0.05
  
                if center[1] < .45:
                    if self.get_depth() > 0.5:
                        msg.axes[const.AXES['vertical']] =  0.2
                    #else no change - don't want to go less than half a meter below the surface
                elif center[1] > .55:
                    msg.axes[const.AXES['vertical']] = -0.2
                if detection:
                    if self.getDistance(detection.box[0], detection.box[1], detection.box[2], detection.box[3]) > 0.4:
                        self.is_close = True

            if self.is_close:
                self.is_close = False
                return "Approached_Gate" # Transitions to INTERACT_GATE
            elif (rospy.get_time() - self.last_seen) > 2:
                msg.axes[const.AXES['frontback']] = 0
                self.joy_pub.publish(msg)
                rospy.logwarn("Lost tracking the gate for more than 2 seconds")
                
                if(gbl.debug):
                    return "Approached_Gate" # DEBUG Purposes Only!
                
                return "Lost_Gate" # Transitions to SEARCH_FRONT_GATE

            self.joy_pub.publish(msg)

            rospy.sleep(const.SLEEP_TIME)

    def log(self):
        rospy.loginfo('Executing state TRACK_GATE')
