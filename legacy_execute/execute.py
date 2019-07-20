import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy
from sensor_msgs.msg import FluidPressure
from mavros_msgs.msg import VFR_HUD
import numpy as np
import time
import math
import pymavlink
import random


use_hold_depth = False 
target_depth = 0
search_frames_seen = 0
depth_const = -.5

# box_callback uses completed to track what's done and states to list the possible states
# each state function takes the list of boxes, calculates what the current motor output should be
# builds that message, and returns it to be published by box_callback.
# Each function is responsible for recognizing what triggers a state change and changing current_state to the appropriate value.
# The global start_time helps with this because it can be maintained across many function calls

last_state = None

completed = dict.fromkeys(['start_gate_found', 'start_gate_passed', 'dice_found', 'dice_hit'], False)

current_target = None
searches_run = 0

is_close = False
last_seen = time.time()

def_msg_axes = (-0.01, 0.0, -1.0, 0.0, 0.0, 1.0, 0.0, 0.0)
def_msg_buttons = (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)

class_dict = {"background":0, "path_marker":1, "start_gate":2, 
            "channel":3, "claw":4, "die1":5, 'die2':6, 'die5':7, 'die6':8,
            'roulette_wheel':9, 'red_wheel_side':10, 'black_wheel_side':11,
            'slot_machine':12, 'slot_handle':13, 'r_slot_target':14, 'y_slot_target':15,
            'r_ball_tray':16, 'g_ball_tray':17, 'floating_area':18, 'r_funnel':19,
            'y_funnel':20, 'g_chip_dispenser':21, 'g_chip_plate':22, 'dieX':23, 'g_funnel':24}

axes_dict = {'rotate': 0, 'vertical' : 1, 'lt' : 2, 'leftright' : 3, 'frontback' : 4, 'rt' : 5, 'dpad_h' : 6, 'dpad_v' : 7}
buttons_dict = {'a' : 0, 'b' : 1, 'x' : 2, 'y' : 3, 'lb' : 4, 'rb' : 5, 'back' : 6, 'start' : 7, 'xbox' : 8, 'lstickpress' : 9, 'rstickpress' : 10}

altitude = 0
thrust_base = -0.485 #roughly steady
thrust_mod = -0.1 #times difference in depth

#psensor = 


# create dict of statuses and whether they've been completed
# that is, track what we've done

def get_center(box):
    #currently biased down for start gate
    return ((box[4] +  box[2]) / 2 ,box[5]) 

def init_msg():
    msg = Joy()
    msg.axes = list(def_msg_axes)
    msg.buttons = list(def_msg_buttons)
    return msg

def distance(x1, y1, x2, y2):
    return math.sqrt((x2-x1)**2 + (y2-y1)**2)

def set_axis(msg, axis, val):
    msg.axes[axes_dict[axis]] = val
    return msg

#depth functions
def depth_callback(msg): #(msg)
    global altitude
    altitude = msg.altitude

def get_depth():
    return altitude - init_depth

def hold_depth():
    global target_depth
    depthdiff= get_depth()-target_depth
    thrust_change = depthdiff * thrust_mod
    thrust = thrust_base + thrust_change
    return thrust


# returns list representing bounding box of highest probability of given class
# returns None if no box of class is found
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
    


# track, move toward, and pass through the gate
def track(boxes):
    global current_state
    global current_target
    global start_time
    global is_close
    global target_depth
    global last_seen
    completed['start_gate_found'] = True #change later, once we have more tasks

    msg = init_msg()
    box = get_box_of_class(boxes, current_target)
    if box and box[1] > .3:
        completed['start_gate_found'] = True
        last_seen = time.time()
        center = get_center(box)
        msg.axes[axes_dict['frontback']] = 0.4
        if center[0] < .45:
            msg.axes[axes_dict['rotate']] = 0.1
        elif center[0] > .55:
            msg.axes[axes_dict['rotate']] = -0.1

        if center[1] < .45:
            if get_depth() > 1:
           	    msg.axes[axes_dict['vertical']] = -0.3
            else: 
                msg.axes[axes_dict['vertical']] = depth_const
        elif center[1] > .55:
            msg.axes[axes_dict['vertical']] = -0.7
        else:
            if not use_hold_depth:
                msg.axes[axes_dict['vertical']] = depth_const #replace with hold_depth later
            else:
                msg.axes[axes_dict['vertical']] = hold_depth() #replace with hold_depth later
        
        if distance(box[2], box[3], box[4], box[5]) > 0.4:
            is_close = True

    elif time.time() - last_seen > 2:
        if is_close:
            target_depth = get_depth()
            start_time = time.time()
            current_state = ramming_speed
        else:
            target_depth = get_depth()
            start_time = time.time()
            current_state = search_forward
        if not use_hold_depth:
            msg.axes[axes_dict['vertical']] = depth_const #replace with hold_depth later
        else:
            msg.axes[axes_dict['vertical']] = hold_depth() #replace with hold_depth later

    msg.axes[axes_dict['frontback']] = .4
    return msg

def ramming_speed(boxes):
    global current_state
    global current_target
    global start_time
    global target_depth
    target_depth = get_depth()
    msg = init_msg()
    msg.axes[axes_dict['frontback']] = 0.7

    if (time.time() - start_time) > 10:
        print("headed to the land of dice, motherfuckers")
        current_target = None

        completed['start_gate_passed'] = True
        
        current_state = find_dice
        msg = init_msg()
        start_time = time.time()

    if not use_hold_depth:
        msg.axes[axes_dict['vertical']] = depth_const #replace with hold_depth later
    else:
        msg.axes[axes_dict['vertical']] = hold_depth() #replace with hold_depth later

    return msg

def surface(boxes):
    global current_state
    global current_target
    global start_time

    msg = init_msg()

    if (time.time() - start_time) < 2:
        msg.axes[axes_dict['vertical']] = .3
    
    else:
        rospy.spin() #wait for shutdown

    return msg
            
def find_dice(boxes):
    # basically a copy of track
    # ram into a bunch of dice then surface
    global current_state
    global current_target
    global start_time
    global target_depth
    global is_close
    global target_depth
    global last_seen

    msg = init_msg()
    # find all boxlike things:
    boxes = []
    for box_name in ['die1', 'die2', 'die5', 'die6','dieX']:
        box = get_box_of_class(boxes, class_dict[box_name])
        if box is not None:
            boxes.append(box)
    if len(boxes) > 0:
        box = max(boxes, key=lambda x: distance(x[2], x[3], x[4], x[5])) # find closest (largest) die
    else:
        box = None

    if box and box[1] > .2:
        completed['dice_found'] = True
        is_close += 1
        last_seen = time.time()
        center = get_center(box)
        msg.axes[axes_dict['frontback']] = 0.4

        # proportional control:
        lr_err = center[0] - 0.5
        lr_signal = (-lr_err + 0) * .52 # signal inverted to match joystick
        msg.axes[axes_dict['leftright']] = max(min(lr_signal, 1), -1)

        # if center[0] < .45:
        #     msg.axes[axes_dict['rotate']] = 0.1
        # elif center[0] > .55:
        #     msg.axes[axes_dict['rotate']] = -0.1

        vert_err = center[1] - 0.5
        vert_signal = (-vert_err + -.45) * 2 # signal inverted because image coordinates
        msg.axes[axes_dict['vertical']] = max(min(vert_signal, 1), -1)

        # if center[1] < .45:
        #     msg.axes[axes_dict['vertical']] = -0.7
        # elif center[1] > .55:
        #     msg.axes[axes_dict['vertical']] = -0.3
        # else:
        #     if not use_hold_depth:
        #         msg.axes[axes_dict['vertical']] = -.55 #replace with hold_depth later
        #     else:
        #         msg.axes[axes_dict['vertical']] = hold_depth(current_depth) #replace with hold_depth later
    else:
        if not use_hold_depth:
            msg.axes[axes_dict['vertical']] = depth_const #replace with hold_depth later
        else:
            msg.axes[axes_dict['vertical']] = hold_depth() #replace with hold_depth later

    if completed['dice_found'] and ((time.time() - last_seen) > 10) and (is_close >= 10):
        completed['dice_hit'] = True
        target_depth = get_depth()
        start_time = time.time()
        current_state = surface

    # random < (tangent of 10 degrees)*2:
    #    move left at 1/2 the speed of moving forward
    if random.random() < (0.1763269807*2) and not completed['dice_found']:
        msg.axes[axes_dict['leftright']] = .2 # this is left

    msg.axes[axes_dict['frontback']] = .4
    return msg

#search for the gate if it isn't initially visible
def search_forward(boxes):
    global current_state
    global current_target
    global start_time
    global target_depth
    global search_frames_seen
    msg = init_msg()
    msg.axes[axes_dict['frontback']] = .4
    if get_box_of_class(boxes, current_target):
        if search_frames_seen <= 2:
            search_frames_seen += 1
        else:
            target_depth = get_depth()
            current_state = track
    elif (time.time() - start_time) > 5: #move forward for 3 secs
        target_depth = get_depth()
        start_time = time.time()
        current_state = search_left
        search_frames_seen = 0
    else:
        search_frames_seen = 0
    if not use_hold_depth:
        msg.axes[axes_dict['vertical']] = depth_const #replace with hold_depth later
    else:
        msg.axes[axes_dict['vertical']] = hold_depth() #replace with hold_depth later
    return msg

#search for the gate if it isn't initially visible
def search_left(boxes):
    global current_state
    global current_target
    global start_time
    global target_depth
    global search_frames_seen
    msg = init_msg()
    msg.axes[axes_dict['rotate']] = -.2
    if get_box_of_class(boxes, current_target):
        if search_frames_seen <= 2:
            search_frames_seen += 1
        else:
            target_depth = get_depth()
            current_state = track
    elif(time.time() - start_time) > 2: #rotate for 2 secs
        target_depth = get_depth()
        start_time = time.time()
        current_state = search_right
        search_frames_seen = 0
    else:
        search_frames_seen = 0
    if not use_hold_depth:
        msg.axes[axes_dict['vertical']] = depth_const #replace with hold_depth later
    else:
        msg.axes[axes_dict['vertical']] = hold_depth() #replace with hold_depth later
    
    return msg

def search_right(boxes):
    global current_state
    global current_target
    global start_time
    global target_depth
    global search_frames_seen

    msg = init_msg()
    msg.axes[axes_dict['rotate']] = .2
    if get_box_of_class(boxes, current_target):
        if search_frames_seen <= 2:
            search_frames_seen += 1
        else:
            target_depth = get_depth()
            current_state = track
    elif(time.time() - start_time) > 4: #rotate for 4 secs
        target_depth = get_depth()
        start_time = time.time()
        current_state = search_recenter
        search_frames_seen = 0
    else:
        search_frames_seen = 0

    if not use_hold_depth:
        msg.axes[axes_dict['vertical']] = depth_const #replace with hold_depth later
    else:
        msg.axes[axes_dict['vertical']] = hold_depth() #replace with hold_depth later


    return msg

def search_recenter(boxes): #rotates back to the left
    global current_state
    global current_target
    global start_time
    global target_depth
    global search_frames_seen

    msg = init_msg()
    msg.axes[axes_dict['rotate']] = -.2
    if not use_hold_depth:
        msg.axes[axes_dict['vertical']] = depth_const #replace with hold_depth later
    else:
        msg.axes[axes_dict['vertical']] = hold_depth()
    if get_box_of_class(boxes, current_target):
        if search_frames_seen <= 2:
            search_frames_seen += 1
        else:
            target_depth = get_depth()
            current_state = track
    elif(time.time() - start_time) > 2: #rotate for 2 secs
        start_time = time.time()
        target_depth = get_depth()
        current_state = search_forward
        search_frames_seen = 0
    else:
        search_frames_seen = 0
    return msg


def start(boxes):
    global current_target
    global current_state
    global start_time
    global target_depth
    curr_msg = init_msg()
    curr_msg.axes[axes_dict['vertical']] = -1
    curr_msg.axes[axes_dict['frontback']] = 1
    if time.time() > (start_time + 15):
        if get_box_of_class(boxes, class_dict['start_gate']):
            current_target = class_dict['start_gate']
            current_state = track
        else:
            target_depth = get_depth()
            start_time = time.time()
            current_target = class_dict['start_gate']
            current_state = search_forward
    return curr_msg

#set boxes
def bbox_callback(msg):
    #get multidimensional list of boxes
    boxes = []
    num_boxes = int(msg.data[0])
    for i in range (num_boxes):
       boxes.append(list(msg.data[7 * i + 1: 7 * i + 7]))
    print(boxes)
# 
    # get function
    print(current_state)
    output = current_state(boxes)

    # target_depth = get_depth()main
    # while not rospy.is_shutdowmain
    #     msg = init_msg()
    #     msg.axes[axes_dict['vemain
    #     pub.publish(msg)

    #publish
    pub.publish(output)


#start execution here
#rospy.sleep(20)
current_state = start
start_time = time.time()

rospy.init_node('subdriver', anonymous=True)
pub = rospy.Publisher('joy', Joy, queue_size=2)
depth_sub = rospy.Subscriber('/mavros/vfr_hud', VFR_HUD, depth_callback)
rospy.sleep(.5)
init_depth = altitude

ssd_sub = rospy.Subscriber('ssd_output', Float32MultiArray, bbox_callback)

rospy.spin()
