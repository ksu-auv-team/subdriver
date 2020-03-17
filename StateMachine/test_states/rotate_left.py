#!/usr/bin/env python2

from StateMachine.sub import *
'''
State: ROTATE_LEFT
    - a simple state that makes the sub rotate left using the axes joystick message
    - automatically goes to ROTATE_RIGHT state when done
'''

# define state test_start
class Rotate_Left(Sub):
    def __init__(self):
        smach.State.__init__(self, outcomes=['rotate_right'])

    def execute(self, userdata):
        self.init_state()

        # Initialize joystick message & rotate sub left
        msg = self.init_joy_msg()
        msg.axes[const.AXES['rotate']] = 0.7

        rospy.sleep(const.SLEEP_TIME)
        return "rotate_right"

    def log(self):
        print('Executing state ROTATE_LEFT')