#!/usr/bin/env python2
'''
sub.py implements a common base class for all states used in the SubDriver
state machine.

As currently implemented, the base class does implicitly depend on state data
stored as global vars in gbl, and system data exposed through rospy API.
'''

#external libraries
import rospy
import smach
import math

import numpy as np
import sys, signal

#messages
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy
from sensor_msgs.msg import FluidPressure
from mavros_msgs.msg import VFR_HUD

from submarine_msgs_srvs.msg import Detections
from classes import Detection

# Global Variables
from StateMachine import gbl
from StateMachine import const





def signal_handler(signal, frame):
    ''' Handles interrupt signals from OS.

    Args:
      signal: signal received from system
      frame: unused? assumed a state capture for diagnostic porpises.
    '''
    rospy.loginfo("\nShutting Down Run...")
    sys.exit(0)

#ROS callbacks
#global to make the linter happy
def vfr_hud_callback(msg): 
    gbl.depth = msg.altitude
    gbl.heading = msg.heading

def bbox_callback(msg):
    gbl.detections = []
    gbl.num_detections = msg.detected[0]

    for i in range(int(gbl.num_detections)):
        detection = Detection.Detection(msg.scores[i], msg.boxes[(i*4):(i+1)*4], msg.classes[i])
        gbl.detections.append(detection)


class Sub(smach.State):
    '''This is our overall 'sub' state. It is the superclass that all the other
    states inherit from.
    
    The idea here being that there are many things that all the states should
    be able to do, but we don't want to re-write them all.
    '''
    def __init__(self):
        '''Initializes a Sub.

        Every state must be initiailized with an __init__ function that defines
        what the outcomes of the state can be. The outcomes determine what
        state is moved to next.
        '''
        smach.State.__init__(self, outcomes=['Finished_Run'])

    def init_state(self):
        '''Initializes a Sub's state using global vars.
        Upon adding the states to the state machine, they run the init_state,
        but we only want them to run it after we have actually started the run.

        Implicit args: gbl.run_start_time, used to check if run is actually
                           started.
                       gbl.depth, current sub depth at start of state.
                       (time), time as returned by rospy.get_time() at start
                           of state.
        '''
        if gbl.run_start_time:
            self.current_state_start_time = rospy.get_time()
            self.current_state_start_depth = gbl.depth
            self.log()


    def move_distance(self, distance, direction):
      """ Direcion will be something like: 'leftright', 'vertical', or 'rotate' 
      This function will be fully implemented once data on acceleration is measured in pool tests """
      pass


    def execute(self, userdata):
        '''Executes the behavior defined for a given state.
        Every state requires an 'execute' function. This is the function that automatically
        gets called by SMACH when we transition into the new state. In each of the subclasses 
        that inherit from 'sub' we override the 'execute' function to do that state's speciffic job.

        Args:
            userdata: dict, k-v mapping of data pertinent to state.
        '''
        pass

    def log(self):
      '''Logs to ROS.

      The log function is designed to be overridden in each subclass as a
      catch-all for when you want to log things to ROS.
      '''
      pass

    def init_joy_msg(self):
      '''This initializes a joystick message. We drive our sub by simulating
      joystick commands and sending them to the Pixhawk.

      Returns:
        empty joystick message ready for editing.
      '''
      msg = Joy()
      msg.axes = list(const.DEFAULT_MSG_AXES)
      msg.buttons = list(const.DEFAULT_MSG_BUTTONS)
      return msg

    def get_center(self, box):
        '''Gets the center point of a bounding box.

        Args:
          box: Box(type?), box whose center to calculate.
        
        Returns:
          center of a bounding box sent to it
        '''
        print(box)
        return ((box[0] + box[2]) / 2.0 , (box[1] + box[3]) / 2.0)

    def get_distance(self, x1, y1, x2, y2):
        '''Gets distance between two points.
        Args:
            x1,y1,x2,y2: float, scalar coordinates for the two points.

        Returns:
            float, distance between the two points.
        '''
        return math.sqrt((x2-x1)**2 + (y2-y1)**2)

    def get_distance_from_box(self, box):
        '''Gets distance between the corners of a bounding box.

        Args:
            x1,y1,x2,y2: float, scalar coordinates for the two points.

        Returns:
            float, distance between the two points.
        '''
        return math.sqrt((box[4]-box[2])**2 + (box[5]-box[3])**2)

    def get_center_screen_offset(self, box, offsetX, offsetY):
        """         
        Args:
            box: [top-left x, top-left y, bottom-right x, bottom-right y]

            offsetX: Horizontal offset as a percentage of the screen's width

            offsetY: Vertical offset as a percentage of the screen's height

        Returns:
            a Joystick message to center the sub around the center of a bounding box with
            offsets relative to percentages of the boxes dimensions.
        """
        STRAFE_LEFTRIGHT_SPEED = 0.3
        VERTICAL_SPEED = 0.3
        # screenWidth = 1.0
        # screenHeight = 1.0

        boxWidth = box[2] - box[0] # box[right] - box[left]
        boxHeight = box[3] - box[1] #box[bottom] - box[top]
        center = self.get_center(box)
        msg = self.init_joy_msg()

        # Horizontal
        if(offsetX > center[0]): # Box is to the left of targetX
            msg.axes[const.AXES['leftright']] = STRAFE_LEFTRIGHT_SPEED # Move Right

        elif(offsetX < center[0]): # Box is to the right of targetX
            msg.axes[const.AXES['leftright']] = -1 * STRAFE_LEFTRIGHT_SPEED # Move Left
        
        
        # Vertical
        if(offsetY > center[1]): # Box is below targetY
            msg.axes[const.AXES['leftright']] = STRAFE_LEFTRIGHT_SPEED # Move Up

        elif(offsetY < center[1]): # Box is above targetY
            msg.axes[const.AXES['leftright']] = -1 * STRAFE_LEFTRIGHT_SPEED # Move Down
        
        return msg


    def get_center_box_offset(self, box, offsetX, offsetY):
        """ 
        Args:
            box: [top-left x, top-left y, bottom-right x, bottom-right y]

            offsetX: Horizontal offset as a percentage of the box's width

            offsetY: Vertical offset as a percentage of the box's height

        Returns:
            a Joystick message to center the sub around the center of a bounding box with
            offsets relative to percentages of the boxes dimensions.
        """
        STRAFE_LEFTRIGHT_SPEED = 0.3
        VERTICAL_SPEED = 0.3


        boxWidth = box[2] - box[0] # box[right] - box[left]
        boxHeight = box[3] - box[1] #box[bottom] - box[top]
        center = self.get_center(box)
        msg = self.init_joy_msg()
        
        relativeX = center[0] + offsetX * boxWidth # Target x position relative to current
        relativeY = center[1] + offsetY * boxHeight # Target y position relative to current

        # Horizontal
        if(relativeX != 0.5):

            if(relativeX > 0.5): # Target Position is to the right
                msg.axes[const.AXES['leftright']] = STRAFE_LEFTRIGHT_SPEED
            elif(relativeX < 0.5): # Target Position is to the left
                msg.axes[const.AXES['leftright']] = -1 * STRAFE_LEFTRIGHT_SPEED
        
        # Vertical
        if(relativeY != 0.5):

            if(relativeY > 0.5): # Target Position is to the below
                msg.axes[const.AXES['vertical']] = -1 * VERTICAL_SPEED
            elif(relativeY < 0.5): # Target Position is to the above
                msg.axes[const.AXES['vertical']] = VERTICAL_SPEED
        
        return msg

    def get_depth(self):
        return gbl.depth - gbl.init_depth

    def get_heading(self):
        return gbl.heading

    #TODO: update this to read in by the new message structure
    def get_box_of_class(self, detections, class_num):
        if gbl.detections == []:
            rospy.loginfo('No detections in image at time: ' + str(rospy.get_time()))
            return None

        found = None
        max_prob = 0.0
        for detection in detections:
            if detection.class_id == class_num and detection.score > max_prob:
                found = detection
                max_prob = detection.score 

        if found:
            rospy.loginfo('class: %s\tconf: %s', str(found.class_id), str(found.score))

        #ignore ghosts
        if max_prob > 0.20:
            return found
        else:
            return None
        
    def set_active_launcher(self):
        '''Changes active launcher. 
            Assumes that the first launcher will be LAUNCHER_LEFT and that the function will be called every time a launcher is used to select the next one.
        '''
        if self.active_launcher == 'LAUNCHER_LEFT':
            self.active_launcher = 'LAUNCHER_RIGHT'
            self.active_launcher_offset = const.LAUNCHER_RIGHT_OFFSET
            return
        else:
            self.active_launcher = None
            self.active_launcher_offset = None
            return

    # These get set at the start of each state, allowing the user to call them
    # as needed
    current_state_start_time = None
    current_state_start_depth = None

    signal.signal(signal.SIGINT, signal_handler)

    search_frames_seen = 0
    last_seen = None

    is_close = False
    init_distance = 0

    joy_pub = rospy.Publisher('joy', Joy, queue_size=2)
    network_sub = rospy.Subscriber('network_output', Detections, bbox_callback)
    vfr_hud_sub = rospy.Subscriber('/mavros/vfr_hud', VFR_HUD, vfr_hud_callback) #provides depth and heading

    active_launcher = 'LAUNCHER_LEFT'
    active_launcher_offset = const.LAUNCHER_LEFT_OFFSET

    joy_pub = rospy.Publisher('joy', Joy, queue_size=2)

