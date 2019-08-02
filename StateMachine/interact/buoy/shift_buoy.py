#!/usr/bin/env python
from StateMachine.gbl import *
from StateMachine.const import *
from StateMachine.sub import *
import rospy

'''
State inheriting from sub that will shift from the first buoy to the second buoy.
'''

class Shift_Buoy(Sub):
    def __init__(self):
        smach.State.__init__(self, outcomes=['finished_shifting'])

    def strafe_right(self):
        rospy.loginfo("SHIFT_BUOY: Right")
        msg = self.init_joy_msg()   
        msg.axes[const.AXES['strafe']] = -0.3
        start_strafe = rospy.get_time()
        while(rospy.get_time() - start_strafe < 4):
            self.publish_joy(msg)

    def move_forward(self):
        rospy.loginfo("SHIFT_BUOY: Forward")
        msg = self.init_joy_msg()
        msg.axes[const.AXES['frontback']] = 0.3        
        start_forward = rospy.get_time()
        while(rospy.get_time() - start_forward < 5):
            self.publish_joy(msg)

    def spin_around(self):
        rospy.loginfo("SHIFT_BUOY: Spin")
        target_heading = gbl.state_heading + 180
        while(abs(self.angle_diff(gbl.heading, target_heading)) < 2):
            msg = self.center_on_heading(target_heading, msg)

    def execute(self, userdata):
        rospy.loginfo('Executing state SHIFT_BUOY')
        # At this point, the sub is stationary and facing the Buoy
        self.init_state()
        gbl.state_heading = gbl.heading

        # Go right
        self.strafe_right()

        # Go forward
        self.move_forward()

        # Spin around
        self.spin_around()

        # Shift right
        self.strafe_right()

    def log(self):
        rospy.loginfo("Executing state SHIFT_BUOY")