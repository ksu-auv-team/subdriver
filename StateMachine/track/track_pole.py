#!/usr/bin/env python

from StateMachine.sub import *

# define state track_pole
class track_pole(sub):
    def __init__(self):
      smach.State.__init__(self, outcomes=['Lost_Pole','Approached_Pole'])

    def execute(self, userdata):
      self.init_state()
      self.last_seen = rospy.get_time()

      #control loop
      while(1):
        msg = self.init_joy_msg()
        box = gbl.get_box_of_class(gbl.boxes, gbl.current_target)
        msg.axes[self.axes_dict['vertical']] = gbl.depth_const

        if (box != None) and box[1] > .3:  # If the box is good
          self.last_seen = rospy.get_time()
          center = self.getCenter(box)
          msg.axes[self.axes_dict['frontback']] = 0.4

          if center[0] < 0.45:
            msg.axes[self.axes_dict['rotate']] = 0.1
          elif center[0] > 0.55:
            msg.axes[self.axes_dict['rotate']] = -0.1

          if center[1] < .45:
            if self.get_depth() > 1:
              msg.axes[self.axes_dict['vertical']] = gbl.depth_const + 0.2
            else: 
              msg.axes[self.axes_dict['vertical']] = gbl.depth_const
          elif center[1] > .55:
            msg.axes[self.axes_dict['vertical']] = gbl.depth_const - 0.2
            if box:
              if self.getDistance(box[2], box[3], box[4], box[5]) > 0.4:
                self.is_close = True


        elif (rospy.get_time() - self.last_seen) > 2: # If the box is not good OR we haven't seen it in a while
          msg.axes[self.axes_dict['frontback']] = 0
          self.joy_pub.publish(msg)

          if self.is_close:
            self.is_close = False
            return "Approached_Pole" # Transitions to INTERACT_POLE
          else:
            rospy.logwarn("Lost tracking the pole for more than 2 seconds")
            if(gbl.debug):
              return "Approached_Pole" # DEBUG Purposes Only!
            return "Lost_Pole" # Transitions to SEARCH_FRONT_POLE

        self.joy_pub.publish(msg)

        rospy.sleep(gbl.sleep_time)

    def log(self):
      rospy.loginfo('Executing state TRACK_POLE')