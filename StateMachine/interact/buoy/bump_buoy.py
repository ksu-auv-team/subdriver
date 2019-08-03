#!/usr/bin/env python
from StateMachine.gbl import *
from StateMachine.const import *
from StateMachine.sub import *
import rospy

'''
State inheriting from sub that will move forward to bump the buoy, then move back to get away from it

Very dumb, all it does is drive forward and backward. We don't even look.
'''

class Bump_Buoy(Sub):
    def __init__(self):
        smach.State.__init__(self, outcomes=['bumped_first_buoy','bumped_second_buoy'])

    def execute(self, userdata):
        rospy.loginfo('Executing state BUMP_BUOY')
        # At this point, the sub is stationary and facing the Buoy
        self.init_state()
        
        msg = self.init_joy_msg()
       
        while (rospy.get_time() < self.current_state_start_time + 5):
            #RAMMING SPEED
            msg.axes[const.AXES['frontback']] = 0.15 #I think this is slow enough?
            self.publish_joy(msg)

        bump_time = rospy.get_time()

        while (rospy.get_time() < bump_time + 5):
            #get away from the buoy
            msg.axes[const.AXES['frontback']] = -0.15
            self.publish_joy(msg)

        rospy.loginfo("Done bumping")
        gbl.current_target = None

        if gbl.buoy_num is 1:
            gbl.current_target = const.CLASSES['buoy_draugr']
            return 'bumped_first_buoy'
        elif gbl.buoy_num is 2:
            return 'bumped_second_buoy'





