#!/usr/bin/env python

from StateMachine import const
from sub import Sub, smach, rospy, gbl

# define state start
class start(Sub):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Not_Found_Gate', 'Found_Gate'])
        

    def execute(self, userdata):
        # Set the run start time to the current ros time
        gbl.run_start_time = rospy.get_time() 

        # Initialize the current state
        self.init_state()
        rospy.loginfo("Run Start Time: " + str(gbl.run_start_time))

        # Set the run start depth
        gbl.init_depth = gbl.depth

    	curr_msg = self.init_joy_msg()
    	curr_msg.axes[const.AXES['vertical']] = -1
    	curr_msg.axes[const.AXES['frontback']] = 1

        gbl.current_target = const.CLASSES['start_gate']

        return 'Not_Found_Gate' # Debug Porpoises Only!

        # Control loop
        while(1):
            self.joy_pub.publish(curr_msg)

            if rospy.get_time() > (gbl.run_start_time + 15):
                if self.get_box_of_class(gbl.detections, const.CLASSES['start_gate']):
                    return 'Found_Gate' # Transitions to TRACK_GATE
                else:
                    return 'Not_Found_Gate' # Transitions to SEARCH_FRONT_GATE

            rospy.sleep(const.SLEEP_TIME)

