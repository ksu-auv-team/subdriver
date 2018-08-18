import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy
from mavros_msgs.msg import VFR_HUD
import numpy as np
import time
import math


# box_callback uses completed to track what's done and states to list the possible states
# each state function takes the list of boxes, calculates what the current motor output should be
# builds that message, and returns it to be published by box_callback.
# Each function is responsible for recognizing what triggers a state change and changing current_state to the appropriate value.
# The global start_time helps with this because it can be maintained across many function calls

last_state = None

completed = dict.fromkeys(['start_gate_found', 'start_gate_passed', 'dice_found', 'dice_hit'], False)

current_target = None

is_close = False
last_seen = time.time()

def_msg_axes = (0.0, 0.0, -1.0, 0.0, 0.0, 1.0, 0.0, 0.0)
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
# thrust_base = -0.425 #roughly steady
# thrust_mod = -0.2 #times difference in depth
# surface_pressure = (101325) #1 atm

#psensor = 


# create dict of statuses and whether they've been completed
# that is, track what we've done

def getCenter(box):
    #currently biased down for start gate
    return (box[4] - box[2], box[5] - box[3] + .1)

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
    print(altitude)

def get_depth():
    global altitude
    global init_depth
    return init_depth - current_depth

# def hold_depth(depth):
#     current_depth = (current_pressure - surface_pressure) * 1.019744/10000
#     print('Depth: {}\tPressure: {} Pa'.format(depth, current_pressure))
#     depthdiff= depth-current_depth
#     thrust_change = depthdiff * thrust_mod
#     thrust = thrust_base + thrust_change
#     return thrust


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
    if max_prob > 0.4:
        return found
    else:
        return None
    


# track, move toward, and pass through the gate
def track(boxes):

    global current_state
    global current_target
    global start_time
    global is_close
    global last_seen
    completed['start_gate_found'] = True #change later, once we have more tasks
    is_close = False

    msg = init_msg()
    box = get_box_of_class(boxes, current_target)
    if box and box[1] > .3:
        last_seen = time.time()
        center = getCenter(box)
        msg.axes[axes_dict['frontback']] = 0.4
        if center[0] < .45:
            msg.axes[axes_dict['leftright']] = 0.2
        elif center[0] > .55:
            msg.axes[axes_dict['leftright']] = -0.2

        if center[1] < .45:
            msg.axes[axes_dict['vertical']] = -0.1
        elif center[1] > .55:
            if get_depth() < .5: #never go above 1m deep
                msg.axes[axes_dict['vertical']] = 0.1

        if distance(box[2], box[3], box[4], box[5]) > 0.67:
            is_close = True

    elif time.time() - last_seen > 2:
        if is_close:
            start_time = time.time()
            current_state = ramming_speed
        else:
            start_time = time.time()
            current_state = search_forward

    msg.axes[axes_dict['frontback']] = .4
    return msg

def ramming_speed(boxes):
    global current_state
    global current_target
    global start_time

    msg = init_msg()
    msg.axes[axes_dict['frontback']] = 0.2

    if (time.time() - start_time) > 10:
        print("headed home motherfuckers")
        current_target = None
        completed['start_gate_passed'] = True
        
        current_state = surface
        msg = init_msg()
        start_time = time.time()

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

    

#search for the gate if it isn't initially visible
def search_forward(boxes):
    global current_state
    global current_target
    global start_time
    msg = init_msg()
    msg.axes[axes_dict['frontback']] = .4
    if get_box_of_class(boxes, current_target):
        current_state = track
    elif (time.time() - start_time) > 12: #move forward for 3 secs
        start_time = time.time()
        current_state = search_left
    return msg

#search for the gate if it isn't initially visible
def search_left(boxes):
    global current_state
    global current_target
    global start_time
    msg = init_msg()
    msg.axes[axes_dict['rotate']] = -.2
    if get_box_of_class(boxes, current_target):
            current_state = track
    elif(time.time() - start_time) > 2: #rotate for 2 secs
        start_time = time.time()
        current_state = search_right
    
    return msg

def search_right(boxes):
    global current_state
    global current_target
    global start_time

    msg = init_msg()
    msg.axes[axes_dict['rotate']] = .2
    if get_box_of_class(boxes, current_target):
            current_state = track
    elif(time.time() - start_time) > 4: #rotate for 4 secs
        start_time = time.time()
        current_state = search_recenter


    return msg

def search_recenter(boxes): #rotates back to the left
    global current_state
    global current_target
    global start_time

    msg = init_msg()
    msg.axes[axes_dict['rotate']] = -.1
    if get_box_of_class(boxes, current_target):
            current_state = track
    elif(time.time() - start_time) > 2: #rotate for 2 secs
        current_state = search_forward
    return msg


def start(boxes):
    global current_target
    global current_state
    global start_time
    curr_msg = init_msg()
    
    curr_msg.axes[axes_dict['frontback']] = 1
    if altitude > -.75:
        curr_msg.axes[axes_dict['vertical']] = -.5
    if time.time() > (start_time + 10):
        if get_box_of_class(boxes, class_dict['start_gate']):
            current_target = class_dict['start_gate']
            current_state = track
        else:
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
     #  boxes[i][0] -= 1 #compensate for network weirdness
   # print(boxes)
# 
    # get function
    print(current_state)
    output = current_state(boxes)
    print(output.axes)
    # target_depth = get_depth()main
    # while not rospy.is_shutdowmain
    #     msg = init_msg()
    #     msg.axes[axes_dict['vemain
    #     pub.publish(msg)

    #publish
    pub.publish(output)


#start execution here
#rospy.sleep(20)

start_time = time.time()
current_state = start

rospy.init_node('subdriver', anonymous=True)
pub = rospy.Publisher('joy', Joy, queue_size=2)
depth_sub = rospy.Subscriber('/mavros/vfr_hud', VFR_HUD, depth_callback)

rospy.sleep(1)
init_depth = altitude

ssd_sub = rospy.Subscriber('ssd_output', Float32MultiArray, bbox_callback)

rospy.spin()
