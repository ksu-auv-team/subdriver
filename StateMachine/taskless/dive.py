#!/usr/bin/env python2

from StateMachine.sub import *

# define state surface
class Dive(Sub):
    def __init__(self):
        smach.State.__init__(self, outcomes=['dived'])

    def execute(self, userdata):
        self.init_state()
	
        print('Executing state DIVE')
        msg = self.init_joy_msg()

        while self.get_depth() >= 0.0:
            msg.axes[const.AXES['vertical']] = -0.3
            self.publish_joy(msg)
        break
     return 'dived'
