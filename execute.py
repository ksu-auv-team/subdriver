import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy
import numpy as np
import threading
import time
import math

box_lock = threading.RLock()
def_msg_axes = [0.0, 0.0, -1.0, 0.0, 0.0, 1.0, 0.0, 0.0]
def_msg_buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

class_dict = {"background":0, "path_marker":1, "start_gate":2, 
            "channel":3, "claw":4, "die1":5, 'die2':6, 'die5':7, 'die6':8,
            'roulette_wheel':9, 'red_wheel_side':10, 'black_wheel_side':11,
            'slot_machine':12, 'slot_handle':13, 'r_slot_target':14, 'y_slot_target':15,
            'r_ball_tray':16, 'g_ball_tray':17, 'floating_area':18, 'r_funnel':19,
            'y_funnel':20, 'g_chip_dispenser':21, 'g_chip_plate':22}

# commands = {'frontback' : 0, 'horiz' : 1, 'vertical' : 2, 'rotate' : 3, 'holddepth' : 4}
axes_dict = {'rotate': 0, 'vertical' : 1, 'lt' : 2, 'leftright' : 3, 'frontback' : 4, 'rt' : 5, 'dpad_h' : 6, 'dpad_v' : 7}
buttons_dict = {'a' : 0, 'b' : 1, 'x' : 2, 'y' : 3, 'lb' : 4, 'rb' : 5, 'back' : 6, 'start' : 7, 'xbox' : 8, 'lstickpress' : 9, 'rstickpress' : 10}

# create dict of statuses and whether they've been completed
# that is, track what we've done 
completed = dict.fromkeys(['start_gate_found', 'start_gate_passed', 'dice_found', 'dice_hit'], False)
boxes = []
print(completed)


def init_msg():
    msg = Joy()
    msg.axes = def_msg_axes
    msg.buttons = def_msg_buttons
    return msg

#set boxes
def bbox_callback(msg, args):
    print('called back')
    l = args
    #lock boxes so we can't read from a partially filled array
    l.acquire()
    boxes = []
    num_boxes = int(msg.data[0])
    for i in range (num_boxes):
        boxes.append(msg.data[7 * i + 1: 7 * i + 7])
    l.release()
    print(boxes)


# returns list representing bounding box of highest probability of given class
# returns None if no box of class is found
def get_box_of_class(class_num, l):
    found = None
    max_prob = 0.0
    l.acquire()
    for box in boxes:
        if box[1] == class_num and box[2] > max_prob:
            found = box
            max_prob = box[2]
    l.release()

    #ignore ghosts
    if max_prob > 0.1:
        return found
    else:
        return None

# track, move toward, and pass through the gate
def track_gate(curr_box):
    completed['start_gate_found'] = True
    is_close = False
    while(curr_box):
        msg = init_msg()
        center = getCenter(curr_box)


        msg.axes[axes_dict['updown']] = -0.425

        if center[0] < 45:
            msg.axes[axes_dict['leftright']] = 0.4
        elif center[0] > .55:
            msg.axes[axes_dict['leftright']] = 0.4

        if center[1] < .45:
            msg.axes[axes_dict['updown']] = -0.6
        elif center[1] > .55:
            msg.axes[axes_dict['updown']] = 0.1
        
        curr_box = get_box_of_class(2, box_lock) #check this later

        if math.sqrt(((curr_box[2] + curr_box[4]) ** 2) + ((curr_box[3] + curr_box[5]) ** 2)) > 0.67:
            is_close = True
        
        pub.publish(msg)


    if is_close:
        ramming_speed(10)
        msg = init_msg
        msg.axes[axes_dict['vertical']] = 1
        pub.publish(msg)
        rospy.sleep(1)
        msg = init_msg
        pub.publish(msg)
    else:
        search_gate()
    pub.publish(msg)

def ramming_speed(duration):
    msg = init_msg()
    msg.axes[axes_dict['frontback']] = 0.4
    msg.axes[axes_dict['vertical']] = -0.425
    for i in range(int(duration)):
        pub.publish(msg)
        rospy.sleep(1)
    completed['start_gate_passed'] = True
    #later

#search for the gate if it isn't initially visible
def search_gate():
    msg = init_msg()
    #move forward for like 2 seconds
    curr_box = get_box_of_class(class_dict['start_gate'], box_lock)
    startTime = time.time()
    currentTime = time.time()
    msg.axes[axes_dict['vertical']] = -.425
    while(currentTime-startTime < 5):
        currentTime = time.time()
        curr_box = get_box_of_class(class_dict['start_gate'], box_lock)
        if curr_box:
            break
        else:
            #go forward
            msg.axes[axes_dict['frontback']] = .4
            rospy.sleep(.2)
    
    while(not curr_box):
        msg.axes[axes_dict['rotate']] = -0.2
        pub.publish(msg)
        rospy.sleep(1.0)
        if get_box_of_class(class_dict['start_gate'], box_lock):
            break
        msg.axes[axes_dict['rotate']] = 0.2
        pub.publish(msg)
        rospy.sleep(2.0)
        if get_box_of_class(class_dict['start_gate'], box_lock):
            break
        msg.axes[axes_dict['rotate']] = -0.2
        pub.publish(msg)
        rospy.sleep(1.0)
        curr_box = get_box_of_class(class_dict['start_gate'], box_lock)
        msg = init_msg()
        pub.publish(msg)

def getCenter(box):
    return (box[4] - box[2], box[5] - box[3])

def start():
    curr_msg = init_msg()
    curr_msg.axes[axes_dict['vertical']] = -1
    curr_msg.axes[axes_dict['frontback']] = 1
    pub.publish(curr_msg)
    rospy.sleep(5)
    curr_msg = init_msg()
    pub.publish(curr_msg)
    curr_box = get_box_of_class(class_dict['start_gate'], box_lock)
    if curr_box:
        track_gate(curr_box)
    else:
        search_gate()

def main():
    rospy.init_node('subdriver', anonymous=True)
    #add killswitch check here
    rospy.sleep(20)

    start()
    rospy.spin()


pub = rospy.Publisher('joy', Joy, queue_size=2)
listen = rospy.Subscriber('ssd_output', Float32MultiArray, bbox_callback, box_lock)

if __name__ == '__main__':
    main()
