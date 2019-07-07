#Global Variables:
run_start_time = None
depth = None #current depth in meters
init_depth = None #depth at beginning of run - should be near 0
depth_const = -0.01 #downward thrust to maintain depth without a depth sensor or other reference point
heading = None #current compass heading in degrees from 0-360
init_heading = None #compass heading at beginning of run
boxes = [] #list of bounding boxes that will be filled by the neural network
current_target = None #the current object being targeted TODO: allow multiple targets
sleep_time = 0.05 #the amount of time to sleep before looking for another frame
debug = False