#! /usr/bin/env python

#Explicit imports help understand what comes from where (and makes linter happy)
from StateMachine.sub import sub
import rospy
import smach
import gbl

class track_torpedo(sub):
  '''Tracker for torpedo targets.

  Maintains sub position relative to torpedo target,
  applying necessary windage adjustments for the ready tube.
  '''
  def __init__(self):
    smach.State.__init__(self, outcomes=['Target_Lost','Target_Locked','Hardware_Failure'])
  
  def execute(self, userdata):
    self.init_state()
    self.last_seen = rospy.get_time()

    while(1):
      jmsg = self.init_joy_msg()
      box = gbl.get_box_of_class(gbl.boxes, gbl.current_target)
      jmsg.axes[self.axes_dict['vertical']] = gbl.depth_const

      if (box is not None) and box[1] > 0.3:
        pass # do something useful


  def log(self):
    rospy.loginfo('Executing state TRACK_TORPEDO')