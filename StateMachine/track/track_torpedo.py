#! /usr/bin/env python

#Explicit imports help understand what comes from where (and makes linter happy)
from StateMachine.const import const
from StateMachine.controllers import PID
from StateMachine.sub import sub
from StateMachine.sub import rospy
from StateMachine.sub import smach
from StateMachine.sub import gbl

import math

class track_torpedo(sub):
  '''Tracker for torpedo targets.

  Maintains sub position relative to torpedo target,
  applying necessary windage adjustments for the ready tube.
  '''
  def __init__(self):
    smach.State.__init__(self, outcomes=['Target_Lost','Target_Locked','Hardware_Failure'])

  def execute(self, userdata):
    # Fail Fast
    if not self.active_launcher:
        rospy.loginfo('[TRACK_TORPEDO] - %s' % ('No available launch tubes'))
        return 'Hardware_Failure'
    self.init_state()
    self.last_seen = rospy.get_time()
    #TODO: Tune these controllers
    x_pid = PID(s=CAMERA_FORWARD_CENTER['X'] + self.active_launcher_offset['WINDAGE_OFFSET'])
    z_pid = PID(s=CAMERA_FORWARD_CENTER['Z'] + self.active_launcher_offset['ELEVATION_OFFSET'] + gbl.depth_const)

    while(1):
      jmsg = self.init_joy_msg()
      box = gbl.get_box_of_class(gbl.boxes, gbl.current_target)

      if (box is not None) and box[1] > 0.3:
        x,z = self.getCenter(box)
        if (abs(CAMERA_FORWARD_CENTER['x']-x) < 5 and abs(CAMERA_FORWARD_CENTER['z']-z) < 5):
          return 'Target_Locked'
        jmsg.axes[AXES_ENUM['leftright']] = x_pid.Update(x)
        jmsg.axes[AXES_ENUM['vertical']] = z_pid.Update(z)
        self.joy_pub.publish(jmsg)


  def log(self):
    rospy.loginfo('Executing state TRACK_TORPEDO')