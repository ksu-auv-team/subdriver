#!/usr/bin/env python2

from StateMachine.sub import *
import random

'''
interact_octagon.py
Implements the Interact_Octagon class, which inherits from Sub

Will interact with the octagon/coffin surfacing task

Works by centering the sub over the coffin, then climbing while keeping it centered
until we hit the surface. If we lose the coffin, it'll keep going straight up until we hit depth 0 or so.

Will try to thrust just past the surface, in case our depth sensor is miscalibrated, then stop.
I'm expecting we won't do anything after we surface - we're allowed to go back under, but I think this will
be the last thing we do.

I thought about trying to look for the octagon on the surface as we climb, but decided against it.
It'll be hard to see on the surface, it doesn't tell us much about horizontal position, and the
depth sensor is more accurate for vertical position.

Variables:
is_centered - whether we've gotten centered over the coffin. We won't start climbing until we are.
thrust_start_time - the time we started the final thrust up.
'''

# define state interact_pole
class Interact_Octagon(Sub):
    self.is_centered = False
    self.thrust_start_time = None


    def __init__(self):
        smach.State.__init__(self, outcomes=['surfaced'])

    def execute(self, userdata):
        #initialization
        self.init_state()

        while (depth < -0.01): #10cm seems like a reasonable (generous) margin of error
            #TODO: update this to how we actually do it
            coffin_center = bottom_camera.get_object_center(coffin)

            if (coffin_center[0] > 0.45 and coffin_center[0] < 0.55 and coffin_center[1] > 0.45 and coffin_center[1] < 0.55):
                self.is_centered = True

            #TODO: check direction for these
            if (coffin_center[0] < 0.45):
                msg.axes[const.AXES['leftright']] = 0.2
            else if (coffin_center[0] > 0.55):
                msg.axes[const.AXES['leftright']] = -0.2
            
            if (coffin_center[1] < 0.45):
                msg.axes[const.AXES['frontback']] = 0.2
            else if (coffin_center[1] > 0.55):
                msg.axes[const.AXES['frontback']] = -0.2
            
            #intent is that if we're centered and lose the coffin we keep going straight up
            if (is_centered):
                msg.axes[const.AXES['vertical']] = 0.2

            self.publish_joy(msg)
        #end while

        self.thrust_start_time = rospy.get_time()

        #thrust up a bit more to be sure we break the surface
        while(rospy.get_time() < self.thrust_start_time + 1):
            msg.axes[const.AXES['vertical']] = 0.2
            self.publish_joy(msg)

        return "surfaced"
