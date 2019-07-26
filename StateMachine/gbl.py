'''
gbl.py holds global variables for all of subdriver and functions used to access them.

Variables:
    run_start_time
        Time at start of run

    depth
        Current depth in m as determined by depth sensor and sent in VFR_HUD messages.
        Updated by vfr_hud_callback().
    
    init_depth
        Depth at beginning of run, which will be the depth at the surface. We store
        this to account for differences in atmospheric pressure.
    
    heading
        Current compass heading in degrees (from 0-360).
        Updated by vfr_hud_callback()

    init_heading
        Compass heading at beginning of run.
    
    state_heading
        Compass heading at beginning of current state/set of states.
        This is messy and there may be a better way to do it - I'm doing it this way mostly for search patterns that cross multiple states.
    
    detections
        List of detections from the neural network.
        Updated by bbox_callback()
    
    current_target = None
        The object currently being targeted.

    debug
        Are we currently in debug mode?
'''

#Global Variables:
run_start_time = None
depth = None #current depth in meters
init_depth = None #depth at beginning of run - should be near 0
heading = None #current compass heading in degrees from 0-360
init_heading = None #compass heading at beginning of run
state_heading = None
detections = [] #list of detections that will be filled by the neural network
num_detections = 0 #why would we not just use len(detections)?
current_target = None #the current object being targeted TODO: allow multiple targets
debug = False
