#!/usr/bin/env python2

from StateMachine.sub import *


'''
track_octagon is the state implementing tracking for the octagon/coffin/surfacing
task. If either the coffin or octagon is detected, but not both, it moves horizontally
toward them and keeps the same depth. If both are detected, it moves toward them
horizontally and centers itself between them vertically.

The state determines that it is near the octagon either when the octagon and coffin disappear
from the screen after we are sufficiently close to it or when the coffin becomes
visible on the bottom camera (unlikely).

I expect that the coffin will be more reliably visible, but this depends on the construction
of the props. From the rules currently, it looks like the coffin will be large
and nothing distinctive will be attached to the octagon. It may be better to look only for
the coffin and/or to bias the sub down so it can see the coffin better - we can always move
back up once we get to it.

'''

# define state track_gate
class track_gate(sub):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Lost_Octagon','Approached_Octagon'])

    def execute(self, userdata):
      self.init_state()
      self.last_seen = rospy.get_time()

      #control loop
      while(1):
        msg = self.init_joy_msg()
        #change to use multiple targets (i.e. coffin and octagon)
        box = gbl.get_box_of_class(gbl.boxes, gbl.current_target)
        msg.axes[self.axes_dict['vertical']] = gbl.depth_const

        

        #if not coffin detected in bottom camera
            #check detected front camera objects to see which we have
            #if any
                #if both
                    #move toward them
                    #move toward point on screen between top of coffin
                    #and bottom of octagon
                #else
                    #if coffin
                        #move toward it
                        #hold depth
                        #maybe if very far above coffin move down, or vice versa?
                    #if octagon
                        #move toward it
                        #hold depth
                        #maybe if very far below octagon move up, or vice versa?
            #if neither
                #move slowly forward and wait for the next box
                #if we can't see either for more than ~1.5 seconds 
                    #if we're close
                        #next state
                    #else
                        # go back to searching
        #else
            #next state

        self.joy_pub.publish(msg)

        rospy.sleep(gbl.sleep_time)

	def log(self):
		rospy.loginfo('Executing state TRACK_GATE')