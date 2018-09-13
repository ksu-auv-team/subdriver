#!/usr/bin/env python

import rospy
import smach
import smach_ros

from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy
from sensor_msgs.msg import FluidPressure
from mavros_msgs.msg import VFR_HUD
import numpy as np
import pymavlink


# define state Foo
class sub(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Finished_Run'])

    def init_state(self):
    	self.state_start_time = rospy.get_time()
    	self.log()

    def execute(self, userdata):
    	pass

    def log(self):
    	pass

    def get_depth(self):
    	pass

    def depth_hold(self):
    	pass    	    	

    def depth_callback(msg): 
    	altitude = msg.altitude

    def init_joy_msg(self):
    	msg = Joy()
    	msg.axes = list(self.def_msg_axes)
    	msg.buttons = list(self.def_msg_buttons)
    	return msg

	def get_box_of_class(boxes, class_num):
		found = None
    	max_prob = 0.0
    	for box in boxes:
        	if box[0] == class_num and box[1] > max_prob:
        		found = box
        		max_prob = box[1] 
    	print('class: ' + str(box[0]) + '\tconf: ' + str(box[1]))

    	#ignore ghosts
    	if max_prob > 0.20:
    		return found
    	else:
        	return None      	

    def bbox_callback(msg):
    	#get multidimensional list of boxes
    	self.boxes = []
    	num_boxes = int(msg.data[0])
    	for i in range (num_boxes):
       		boxes.append(list(msg.data[7 * i + 1: 7 * i + 7]))
    	#print(boxes)

    	# get function
    	#print(current_state)
    	#output = current_state(boxes)

    	# target_depth = get_depth()main
    	# while not rospy.is_shutdowmain
    	#     msg = init_msg()
    	#     msg.axes[axes_dict['vemain
    	#     pub.publish(msg)

    	#publish
    	#joy_pub.publish(output) 




    boxes = []
    current_target = None

    run_start_time = None
    state_start_time = None

    joy_pub = rospy.Publisher('joy', Joy, queue_size=2)
    ssd_sub = rospy.Subscriber('ssd_output', Float32MultiArray, bbox_callback)
    depth_sub = rospy.Subscriber('/mavros/vfr_hud', VFR_HUD, depth_callback)

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

    altitude = None