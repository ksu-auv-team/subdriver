#!/usr/bin/env python
import rospy
import smach
import smach_ros
import math

from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy
from sensor_msgs.msg import FluidPressure
from mavros_msgs.msg import VFR_HUD
import numpy as np
import pymavlink

# Global Vaiables
import gbl

import sys, signal

def signal_handler(signal, frame):
    print("\nShutting Down Run...")
    sys.exit(0)

# This is our overall 'sub' state. It is the superclass that all the other states inherit from.
# The idea here being that there are many things that all the states should be able to do, 
# but we don't want to re-write them all.
class sub(smach.State):
    # Every state must be initiailized with an __init__ function that defines what the outcomes
    # of the state can be. The outcomes determine what state is moved to next.
    def __init__(self):
        smach.State.__init__(self, outcomes=['Finished_Run'])

    def init_state(self):
        #Upon adding the states to the state machine, they run the init_state,
        #but we only want them to run it after we have actually started the run
        if gbl.run_start_time:
            self.current_state_start_time = rospy.get_time()
            self.current_state_start_altitude = gbl.altitude
            self.log()

    # Every state requires an 'execute' function. This is the function that automatically
    # gets called by SMACH when we transition into the new state. In each of the subclasses 
    # that inherit from 'sub' we override the 'execute' function to do that state's speciffic job.
    def execute(self, userdata):
    	pass

    # The log function is designed to be overridden in each subclass as a catch-all for when 
    # you want to log things to ROS.
    def log(self):
    	pass

    # This initializes a joystick message. We drive our sub by simulating joystick commands and sending
    # them to the Pixhawk. This function will return to you an empty joystick message ready to be edited.
    def init_joy_msg(self):
    	msg = Joy()
    	msg.axes = list(self.def_msg_axes)
    	msg.buttons = list(self.def_msg_buttons)
    	return msg
    
    # The depth_hold does what is says: holding the depth. It compares the current altitude 'gbl.altitude'
    # to what the altitude was at the start of the current state. If it's higher or lower, it adjusts 
    # 'thrust' which gets returned. The one thing to keep in mind here, is that depth_hold is not actually 
    # commanding your sub anything, just returning the value to pack into your message to hold the current depth.
    def depth_hold(self):
        msg = self.init_joy_msg()

        if gbl.altitude == None or self.current_state_start_altitude == None:
            thrust = gbl.depth_const
            rospy.logerr("While trying to hold altitude, altitude = None")

        elif gbl.altitude - self.current_state_start_altitude > 0.25:
            if gbl.get_depth() > 1:
                thrust = gbl.depth_const + 0.2
            else: 
                thrust = gbl.depth_const
        elif gbl.altitude - self.current_state_start_altitude < -0.25:
            thrust = gbl.depth_const - 0.2
        else:
            thrust = gbl.depth_const

        return thrust

    # Returns the center of a bounding box sent to it
    def getCenter(self, box):
        return ((box[4] +  box[2]) / 2 ,box[5])

    # Returns the distance between two points sent
    def getDistance(self, x1, y1, x2, y2):
        return math.sqrt((x2-x1)**2 + (y2-y1)**2)

    # These get set at the start of each state, allowing the user to call them as needed
    current_state_start_time = None
    current_state_start_altitude = None

    signal.signal(signal.SIGINT, signal_handler)

    search_frames_seen = 0
    last_seen = None

    is_close = False

    joy_pub = rospy.Publisher('joy', Joy, queue_size=2)

    def_msg_axes = (-0.01, 0.0, -1.0, 0.0, 0.0, 1.0, 0.0, 0.0)
    def_msg_buttons = (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)

    axes_dict = {'rotate': 0, 'vertical' : 1, 'lt' : 2, 'leftright' : 3, 'frontback' : 4, 'rt' : 5, 'dpad_h' : 6, 'dpad_v' : 7}
    buttons_dict = {'a' : 0, 'b' : 1, 'x' : 2, 'y' : 3, 'lb' : 4, 'rb' : 5, 'back' : 6, 'start' : 7, 'xbox' : 8, 'lstickpress' : 9, 'rstickpress' : 10}

    class_dict = {"background":0, "path_marker":1, "start_gate":2, 
            "channel":3, "claw":4, "die1":5, 'die2':6, 'die5':7, 'die6':8,
            'roulette_wheel':9, 'red_wheel_side':10, 'black_wheel_side':11,
            'slot_machine':12, 'slot_handle':13, 'r_slot_target':14, 'y_slot_target':15,
            'r_ball_tray':16, 'g_ball_tray':17, 'floating_area':18, 'r_funnel':19,
            'y_funnel':20, 'g_chip_dispenser':21, 'g_chip_plate':22, 'dieX':23, 'g_funnel':24}