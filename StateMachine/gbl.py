import rospy
from std_msgs.msg import Float32MultiArray
from mavros_msgs.msg import VFR_HUD

'''
gbl.py holds global variables for all of subdriver and functions used to access them.

Variables:
    run_start_time
        Time at start of run

    depth
        Current depth in m as determined by depth sensor and sent in VFR_HUD messages.
        Updated by depth_callback().
    
    init_depth
        Depth at beginning of run, which will be the depth at the surface. We store
        this to account for differences in atmospheric pressure.
    
    depth_const = -0.5
        Constant motor thrust down that will maintain roughly the same depth.
        This is only an approximation; it's much better to use the depth sensor.
    
    boxes
        List of bounding boxes from the neural network.
        Updated by bbox_callback()

    bottom_boxes
        List of bounding boxes from bottom camera identified by the neural network.
    
    current_target = None
        The object currently being targeted.

    sleep_time = 0.05
        Constant amount of time to sleep before checking boxes again.
        Used to keep loops from constantly running unnecessarily.

    ssd_sub
        ROS subscriber to neural network.

    depth_sub
        ROS subscriber to Pixhawk VFR_HUD messages.
'''

#TODO: Add support for multiple sets of bounding boxes, i.e. front and bottom cameras


#callbacks
'''
ROS callback that sets depth from VFR_HUD messages.

Parameters:
    msg - VFR_HUD message from ROS
'''
def depth_callback(msg): 
    depth = msg.altitude

'''
ROS callback run every time a front camera frame is received from the neural network. Breaks
the message data into 

Parameters:
    msg - Float32MultiArray message from ROS
'''
def bbox_callback(msg):
    #get multidimensional list of boxes
    boxes = []
    num_boxes = int(msg.data[0])
    for i in range (num_boxes):
        boxes.append(list(msg.data[7 * i + 1: 7 * i + 7]))
    #rospy.loginfo(boxes)

'''
ROS callback run every time a bottom camera frame is received from the neural network. Breaks
the message data into 

Parameters:
    msg - Float32MultiArray message from ROS
'''
def bottom_box_callback(msg):
    #get multidimensional list of boxes
    bottom_boxes = []
    num_boxes = int(msg.data[0])
    for i in range (num_boxes):
        bottom_boxes.append(list(msg.data[7 * i + 1: 7 * i + 7]))
    #rospy.loginfo(boxes)

#global functions
'''
Gets current depth by subtracting altitude from VFR_HUD messages from the initial depth,
which will be detected at the surface. Use this to get current depth instead of accessing
depth directly.

Returns:
    depth - floating-point value describing depth in meters
'''
def get_depth():
        return depth - init_depth

'''
Takes a list of bounding boxes from the neural network and returns the box in which
an object of class class_num was found with the highest confidence. Bounding boxes
are currently implemented as a list of floating-point numbers. 

Parameters:
    boxes - list of bounding boxes (implemented as lists)
    class_num - ID of class to detect
    confidence - Probability confidence needed to ID an object. Should be between 0 and 1.

Returns:
    None or a bounding box (list)
'''
def get_box_of_class(boxes, class_num, confidence = 0.30):
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
    
    rospy.loginfo('class: %s\tconf: %s', box[0], box[1])

    #ignore ghosts
    if max_prob > confidence:
        return found
    else:
        return None


'''
More versatile, but slower version of get_box_of_class that takes multiple classes and returns
a list of all boxes in which any of them are found.

Parameters:
    boxes - list of bounding boxes (implemented as lists)
    class_num - List of IDs of classes to detect
    confidence - Probability confidence needed to ID an object. Should be between 0 and 1.

Returns:
    List of bounding boxes (which are themselves lists). Empty list if none found.
'''
def get_boxes_of_classes(boxes, classes, confidence = 0.30):
    found_boxes = []

    if boxes == []:
        rospy.sleep(1)
        rospy.loginfo('No boxes in image at time: ' + str(rospy.get_time()))
        return None

    for box in boxes:
        for class_num in classes:
            if box[0] == class_num and box[1] > confidence:
                found_boxes.append(box)
    
    rospy.loginfo('class: %s\tconf: %s', box[0], box[1])

    return found_boxes
    

#Global Variables:
run_start_time = None
depth = None
init_depth = None
depth_const = -0.5
boxes = []
bottom_boxes = []
current_target = None
sleep_time = 0.05
ssd_sub = rospy.Subscriber('ssd_output', Float32MultiArray, bbox_callback)
depth_sub = rospy.Subscriber('/mavros/vfr_hud', VFR_HUD, depth_callback)