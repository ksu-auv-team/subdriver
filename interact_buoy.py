#!/usr/bin/env python
from StateMachine.gbl import *
from StateMachine.const import *
from StateMachine.sub import *
from StateMachine.controllers import PID
import math
import time
from enum import Enum

# define state interact_buoy

# Known Information:
    # This won't rotate. Instead the point of the triangle will be pointing toward the gate.
    # The order for each side (and who is next to whom) will be specified.
    # You will get points for touching any of the sides
    # You will get more points for touching the Vampire that you called at the start of your run
    # You will get even more points for touching the Vampire on the backside of the triangle (farthest from the gate, if you specified that Vampire)

# Calculated Information:
    # When to move forward
    # Rotation speed of buoy 
    # Current face on buoy
    # Order of faces (determined by which direction that the buoy is spinnning)

# Needed information:

#Python execute_withState
#logitech camera 1920 x 1080p camera:
    # Logitech web cam C930e


# define state interact_buoy
class Interact_Buoy(Sub):
    #setting to none indicates that we haven't seen it yet
    init_size = None

    def __init__(self):
        smach.State.__init__(self, outcomes=['around_buoy','lost_buoy'])

    def execute(self, userdata):
        #initialization
        self.init_state()
        self.last_seen = rospy.get_time()
        self.maxAcceleration(20)
        init_heading = self.get_heading()

        # Start the front network
        self.use_front_network(True)

        #get initial heading
        while not init_heading:
            rospy.sleep(const.SLEEP_TIME)
            init_heading = self.get_heading()

        #keep going until we're within 10 degrees of the opposite of the initial heading
        while (abs(init_heading - self.get_heading()) < 170 or abs(init_heading - self.get_heading()) > 190):
            detection = self.get_box_of_class(gbl.detections_front, gbl.current_target)
            center = self.get_center(detection.box)
            msg = self.init_joy_msg()

            if detection != None:  # If the box is good
                #update values
                if self.init_size == None:
                    self.init_size = self.get_distance_from_box(detection.box)
                self.last_seen = rospy.get_time()
            elif (rospy.get_time - self.last_seen) <= 5:
                #stay still and look around to see if we can pick it back up
                msg.axes[const.AXES['rotate']] = -0.1 * random.randint(-1, 1)
                self.publish_joy(msg)
                rospy.sleep(const.SLEEP_TIME)
                continue
            else: #if last seen more than 5 seconds ago
                return 'Lost_Buoy' # Transitions to SEARCH_BUOY (I think)

            #strafe right
            msg.axes[const.AXES['leftright']] = 0.15  #rename to 'strafe'

            #keep the buoy centered by rotating
            #these are fast, but I'm assuming we want to make sure rotation keeps up so the circle stays tight.
            #Worst-case is probably that we get jumps of fast rotation followed by nothing.
            if center[0] < 0.45:
                msg.axes[const.AXES['rotate']] = 0.1
            elif center[0] > 0.55:
                msg.axes[const.AXES['rotate']] = -0.1

            #maintain distance
            if self.get_distance_from_box(detection.box) > 1.2 * self.init_size:
                msg.axes[const.AXES['frontback']] = -0.2
            elif self.get_distance_from_box(detection.box) < 0.8 * self.init_size:
                msg.axes[const.AXES['frontback']] = 0.2

            #hold depth
            #if we can see the ends of the buoy (i.e. the bounding box doesn't end at the edge of the screen), center on it
            if detection.box[1] > 0.1 and detection.box[3] < 0.9: #box 1 and 3 are the Y-coordinates
                if (center[1]) < 0.45:
                    msg.axes[const.AXES['vertical']] = 0.1
                elif center[1] > 0.55:
                    msg.axes[const.AXES['vertical']] = -0.1
            #otherwise no change

            self.publish_joy(msg)  
        #end while

        #Hit the buoy
        while(self.get_distance_from_box(detection.box) != 0):
            msg.axes[const.AXES['frontback']] = 0.2
            if detection.box[1] > 0.1 and detection.box[3] < 0.9:
                if (center[1]) < 0.45:
                    msg.axes[const.AXES['vertical']] = 0.1
                elif center[1] > 0.55:
                    msg.axes[const.AXES['vertical']] = -0.1
            if detection.box[0] > 0.1 and detection.box[2] < 0.9:
                if (center[0]) < 0.45:
                    msg.axes[const.AXES['leftright']] = -0.1
                elif center[0] > 0.55:
                    msg.axes[const.AXES['leftright']] = 0.1

            # if detection == None:  # If the box is gone
            #     if a == None:
            #         a = rospy.get_time
            #     elif (rospy.get_time-a) <= 5:
            #         rospy.sleep(2)
        #end while

        time.sleep(2)
        

        while detection.box == (0,0,0,0) or self.get_distance_from_box(detection.box) != self.init_size:
            msg.axes[const.AXES['frontback']] = -0.3
        
        while (abs(init_heading - self.get_heading()) < -10 or abs(init_heading - self.get_heading()) > 10):
            msg.axes[const.AXES['rotate']] = 0.2

        


        #move out of the buoy's way
        #keep strafing without rotating until the buoy is to our side
        #exact position is only a guess and will probably need to be modified
        while(True):
            detection = self.get_box_of_class(gbl.detections_front, gbl.current_target)
            center = self.get_center(detection.box)
            if (center[0] > 0.2 and center[0] < 0.8):
                msg = self.init_joy_msg()
                msg.axes[const.AXES['leftright']] = 0.15
                self.publish_joy(msg)
            else:
                break
        
        gbl.current_target = const.CLASSES['start_gate']

        #headed home
        return 'Around_Bouy' # Transitions to SEARCH_FRONT_GATE





