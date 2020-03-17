#!/usr/bin/env python2

from StateMachine.sub import *
'''
State: ROTATE_RIGHT
    - a simple state that makes the sub rotate right using the axes joystick message
    - automatically goes to SURFACE state when done
'''

# define state test_start
class Rotate_Right(Sub):
    def __init__(self):
        smach.State.__init__(self, outcomes=['surface'])

    def execute(self, userdata):
        self.init_state()

        # Initialize joystick message & rotate sub right
        msg = self.init_joy_msg()
        msg.axes[const.AXES['rotate']] = -0.7

        rospy.sleep(const.SLEEP_TIME)
        return "surface"

    def log(self):
        print('Executing state ROTATE_RIGHT')