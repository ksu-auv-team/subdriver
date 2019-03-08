#!/usr/bin/env python
from StateMachine.constants import BUTTONS_ENUM, JOY_MAP

from StateMachine.sub import sub
from StateMachine.sub import rospy
from StateMachine.sub import smach

import math

class LaunchError(Exception):
    '''Base class for exceptions in this module. Indicates a failure to launch.'''
    pass

class LauncherReadyError(LaunchError):
    '''Indicates that launchers are failing to present as READY.'''
    pass

class interact_torpedo(sub):
    '''Executes interaction state for a task requiring launching torpedoes.
    
    Assumptions:
      - Torpedo launch is mapped to 1 or 2 joystick buttons (left and right
        launchers would need different windage caluclations applied, based on
        distance to target).
      - State is entered from a track_torpedo_target
        such that the desired target is aligned and centered.
    '''
    def __init__(self):
        smach.State.__init__(self, outcomes=['TORPEDO_SUCCESS',
                                             'TORPEDO_FAILURE'])

    def execute(self, userdata):
        '''Executes the INTERACT_TORPEDO state's primariy action.'''
        rospy.loginfo('Executing state INTERACT_TORPEDO')
        if not self.active_launcher:
            rospy.loginfo('[INTERACT_TORPEDO] - %s' % ('No available launch tubes'))
            return 'Torpedo_Failed'
        try:
          self.launch(self.active_launcher)
          return 'Torpedo_Launched'
        except LaunchError as e:
            #Issues with launchers themselves - failure to ready, failure to
            #fire, etc.
            rospy.loginfo('[INTERACT_TORPEDO] - %s' % (e.message))
            return 'Torpedo_Failed'
        except Exception as e:  
            #Some other thing is broken, likely this very code.
            rospy.logwarn('[INTERACT_TORPEDO] - %s' % (e.message))
            return 'Torpedo_Failed'

    def launch(self, launcher):
        '''Launches a torpedo at the target.

        Args:
          launcher: string, id'd launcher, assumed armed and ready to fire.
        Raises:
          LaunchError: if there is an issue with the launcher(s) preventing
          torpedo launch.
        '''
        try:
            jmsg = self.init_joy_msg()
            jmsg.buttons[BUTTONS_ENUM[JOY_MAP[launcher]]]=1
            self.joy_pub.publish(jmsg)
            rospy.sleep(1)
            jmsg.buttons[BUTTONS_ENUM[JOY_MAP[launcher]]]=0
            self.joy_pub.publish(jmsg)
            # Activates next tube
            self.get_launcher()
        except Exception as e:
            raise LaunchError(e)






