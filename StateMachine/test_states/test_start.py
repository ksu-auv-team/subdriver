#!/usr/bin/env python2

from StateMachine.sub import *
'''
State: TEST_START
    - a clone of the original start state but with no object detection
    - automatically goes to ROTATE_LEFT state when done
'''
# define state test_start
class Test_Start(Sub):
    def __init__(self):
        smach.State.__init__(self, outcomes=['rotate_left'])

    def execute(self, userdata):

        self.init_state()

        # Initialize joystick message & submerge sub
        curr_msg = self.init_joy_msg()
        curr_msg.axes[const.AXES['frontback']] = 0.5
        curr_msg.axes[const.AXES['vertical']] = -0.7

        rospy.sleep(const.SLEEP_TIME)

        return "rotate_left"

    def log(self):
        print('Executing state TEST_START')