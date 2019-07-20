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
from sensor_msgs.msg import Health
from sensor_msgs.msg import FluidPressure
from mavros_msgs.msg import VFR_HUD

from submarine_msgs_srvs.msg import Detections
from classes import Detection

# Global Variables
from StateMachine import gbl
from constants import *





def signal_handler(signal, frame):
    ''' Handles interrupt signals from OS.

    Args:
      signal: signal received from system
      frame: unused? assumed a state capture for diagnostic porpises.
    '''
    rospy.loginfo("\nShutting Down Run...")
    sys.exit(0)


class sub(smach.State):
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


    def moveDistance(self, distance, direction):
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
      msg.axes = list(DEFAULT_MSG_AXES)
      msg.buttons = list(DEFAULT_MSG_BUTTONS)
      return msg
    
    def depth_hold(self):
        '''Holds sub depth to value from 'gbl.depth'.
      
        TODO: remove

        Now useless because the Pixhawk is working correctly

        The depth_hold does what is says: holding the depth. It compares the current depth 'gbl.depth'
        to what the depth was at the start of the current state. If it's higher or lower, it adjusts 
        'thrust' which gets returned. The one thing to keep in mind here, is that depth_hold is not actually 
        commanding your sub anything, just returning the value to pack into your message to hold the current depth.
      
        Returns:
            thrust, float(?) value for maintaining the depth to pass along to controller.
        '''
        return 0.0

    def getCenter(self, box):
        '''Gets the center point of a bounding box.

        Args:
          box: Box(type?), box whose center to calculate.
        
        Returns:
          center of a bounding box sent to it
        '''
        print(box)
        return ((box[0] + box[2]) / 2.0 , (box[1] + box[3]) / 2.0)

    def getDistance(self, x1, y1, x2, y2):
        '''Gets distance between two points.

        Args:
            x1,y1,x2,y2: float, scalar coordinates for the two points.

        Returns:
            float, distance between the two points.
        '''
        return math.sqrt((x2-x1)**2 + (y2-y1)**2)
    def getCenterScreenOffset(self, box, offsetX, offsetY):
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
        center = self.getCenter(box)
        msg = self.init_joy_msg()

        relativeX = center[0] + offsetX # Target x position relative to current
        relativeY = center[1] + offsetY # Target y position relative to current

        # Horizontal
        if(relativeX != 0.5):

            if(relativeX > 0.5): # Target Position is to the right
                msg.axes[self.axes_dict['leftright']] = STRAFE_LEFTRIGHT_SPEED
            elif(relativeX < 0.5): # Target Position is to the left
                msg.axes[self.axes_dict['leftright']] = -1 * STRAFE_LEFTRIGHT_SPEED
        
        # Vertical
        if(relativeY != 0.5):

            if(relativeY > 0.5): # Target Position is to the below
                msg.axes[self.axes_dict['vertical']] = -1 *VERTICAL_SPEED
            elif(relativey < 0.5): # Target Position is to the above
                msg.axes[self.axes_dict['vertical']] =  VERTICAL_SPEED
        
        return msg


    def getCenterBoxOffset(self, box, offsetX, offsetY):
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
        center = self.getCenter(box)
        msg = self.init_joy_msg()
        
        relativeX = center[0] + offsetX * boxWidth # Target x position relative to current
        relativeY = center[1] + offsetY * boxHeight # Target y position relative to current

        # Horizontal
        if(relativeX != 0.5):

            if(relativeX > 0.5): # Target Position is to the right
                msg.axes[self.axes_dict['leftright']] = STRAFE_LEFTRIGHT_SPEED
            elif(relativeX < 0.5): # Target Position is to the left
                msg.axes[self.axes_dict['leftright']] = -1 * STRAFE_LEFTRIGHT_SPEED
        
        # Vertical
        if(relativeY != 0.5):

            if(relativeY > 0.5): # Target Position is to the below
                msg.axes[self.axes_dict['vertical']] = -1 * VERTICAL_SPEED
            elif(relativey < 0.5): # Target Position is to the above
                msg.axes[self.axes_dict['vertical']] = VERTICAL_SPEED
        
        return msg
            



        



    # ROS callbacks
    def vfr_hud_callback(msg): 
        gbl.depth = msg.altitude
        gbl.heading = msg.heading

    #TODO: Update this to read in the new Tensorflow message structure
    def bbox_callback(msg):
        gbl.detections = []
        gbl.num_detections = msg.detected[0]

        for i in range(int(gbl.num_detections)):
            detection = Detection.Detection(msg.scores[i], msg.boxes[(i*4):(i+1)*4], msg.classes[i])
            gbl.detections.append(detection)

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
    def getDistance(self, box):
        '''Gets distance between the corners of a bounding box.

        Args:
            x1,y1,x2,y2: float, scalar coordinates for the two points.

        Returns:
            float, distance between the two points.
        '''
        return math.sqrt((box4-box2)**2 + (box5-box3)**2)

    def set_active_launcher(self):
      '''Changes active launcher. 
        Assumes that the first launcher will be LAUNCHER_LEFT and that the function will be called every time a launcher is used to select the next one.
      '''
      if self.active_launcher == 'LAUNCHER_LEFT':
        self.active_launcher = 'LAUNCHER_RIGHT'
        self.active_launcher_offset = LAUNCHER_RIGHT_OFFSET
        return
      else
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
    active_launcher_offset = LAUNCHER_LEFT_OFFSET
    

    joy_pub = rospy.Publisher('joy', Joy, queue_size=2)
