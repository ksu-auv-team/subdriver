import rospy
from std_msgs.msg import Float32MultiArray
from mavros_msgs.msg import VFR_HUD



#callbacks
def depth_callback(msg): 
    depth = msg.altitude

def bbox_callback(msg):
    """
    Values in a Box Array in order:
    #       Each group of 7 values will describe an object/box These 7 values in order.
    #       The values are:
    #         0: image_id (always 0)
    #         1: class_id (this is an index into labels)
    #         2: score (this is the probability for the class)
    #         3: box left location within image as number between 0.0 and 1.0
    #         4: box top location within image as number between 0.0 and 1.0
    #         5: box right location within image as number between 0.0 and 1.0
    #         6: box bottom location within image as number between 0.0 and 1.0 
    """
    #get multidimensional list of boxes
    boxes = []
    num_boxes = int(msg.data[0])
    for i in range (num_boxes):
        boxes.append(list(msg.data[7 * i + 1: 7 * i + 7]))
    #print(boxes)

#global functions

def get_depth():
        return depth - init_depth

def get_box_of_class(boxes, class_num):
    if boxes == []:
        rospy.sleep(1)
        rospy.loginfo('No boxes in image at time: ' + str(rospy.get_time()))
        return None

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

#Global Variables:
run_start_time = None
depth = None
init_depth = None
depth_const = -0.5
boxes = []
current_target = None
sleep_time = 0.05


ssd_sub = rospy.Subscriber('ssd_output', Float32MultiArray, bbox_callback)
depth_sub = rospy.Subscriber('/mavros/vfr_hud', VFR_HUD, depth_callback)

