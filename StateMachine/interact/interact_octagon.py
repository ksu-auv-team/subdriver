#!/usr/bin/env python2

from StateMachine.sub import *

'''
    interact_octagon is the state for interacting with the 
    octagon/coffin/sunlight/surfacing task. It assumes that the sub
    is close to the coffin/octagon and between them in depth when the
    state starts.

    It works by centering the coffin in the bottom camera, stopping
    completely, then moving up while keeping the coffin centered
    until it goes out of view (if it does). If the coffin does go out of
    view, the sub will go straight up until surfaced.

    The best way to determine when the sub is surfaced is unclear. Possibilities
    include: looking at parts of the octagon in the front camera,
    using the depth sensor, or going up for n seconds.

    It ignores the octagon because it's too hard to see.

    We may decide to make this the last state in the run, which is likely,
    or we can implement a following state that will resubmerge the sub and move
    to another task.
'''

# define state interact_gate
class interact_octagon(sub):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Surfaced'])

    def execute(self, userdata):
        self.init_state()

        msg = self.init_joy_msg()

        msg.axes[self.axes_dict['frontback']] = 0.1
        msg.axes[self.axes_dict['vertical']] = self.depth_hold()

        #start by slowly creeping forward until the coffin is visible

        #once the coffin is visible, center the sub over it

        #once the sub is centered, thrust up and stay centered

        #once the sub is near-surfaced/after a max of 5 or so seconds,
        #stop thrusting

        #either stop or resubmerge.
        

    def log(self):
        rospy.loginfo('Executing state interact_octagon')