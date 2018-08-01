import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy
import numpy as np
import threading
import time

box_lock = threading.RLock()
curr_msg_axes = [0.0, 0.0, -1.0, 0.0, 0.0, 1.0, 0.0, 0.0]
curr_msg_buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

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
def get_box_of_class(boxes, class_num, l):
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
    is_close = False
    while(curr_box):
        msg = Joy()
        center = getCenter()

        if center[0] < -.45:
            msg.axes[axes_dict['leftright']] = 0.4
        elif center[0] > .45:
            msg.axes[[axes_dict['leftright']] = 0.4

        if center[1] < -.45:
            msg.axes[axes_dict['updown']] = -0.4
        elif center[1] > .45:
            msg.axes[axes_dict['updown']] = 0.4
        curr_box = get_box_of_class(2) #check this later

        if sqrt(((curr_box[2] + curr_box[4]) ** 2) + ((curr_box[3] + curr_box[5]) ** 2))

    if 

    
    publisher.pub()

#search for the gate if it isn't initially visible
def search(object):
    #move forward for like 2 seconds
    curr_box = get_box_of_class(boxes, class_dict['start_gate'])
    msg.data = Float32MultiArray
    startTime = time.time()
    currentTime = time.time()
    while(currentTime-startTime < 2):
        if curr_box:
            break
        else:
            msg.data = [axes_dict['frontback'], .4]
            currentTime = time.time()
    while(not curr_box):
        msg.data = [axes_dict['rt'], .4]
    pass

def getCenter(box):
    return (box[4] - box[2], box[5] - box[3])

def start():
    while(curr_box)
    curr_box = get_box_of_class(boxes, class_dict['start_gate'], box_lock)
    if curr_box:
        track_gate(curr_box)
    else:
        search(class_dict['start_gate'])

def main():
    rospy.init_node('subdriver', anonymous=True)
    rospy.Publisher('movement_commands', Float32MultiArray, queue_size=2)
    rospy.Subscriber('ssd_output', Float32MultiArray, bbox_callback, box_lock)
    #add killswitch check here

    start()
    rospy.spin()


if __name__ == '__main__':
    main()