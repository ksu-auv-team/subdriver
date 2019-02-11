#!/usr/bin/env python

from collections import namedtuple

from StateMachine.sub import *


class LaunchError(Exception):
    '''Base class for exceptions in this module. Indicates a failure to launch.'''
    pass

class LauncherReadyError(LaunchError):
    '''Indicates that launchers are failing to present as READY.'''
    pass


#TODO(travis): Identify status messages to determine launcher state.

#Kentucky offsets for y-z plane(parallel to target)
Kentucky = namedtuple('Kentucky',['windage','elevation'])
'''Kentucky(Windage) offesets to apply just before launching torpedo.

Defines offset in MoA (or just degrees, depending upon launcher accuracy and
offset required) to compensate for launcher mounting position in relation
to sub Line-of-Sight. Named after the practice in shooting sports where instead
of adjusting the rifle's sights to align point-of-aim to point-of-strike, the
shooter adjusts their point-of-aim to offset the deviation from zero. Not a dig
at Mechanical's ability to get the launcher to shoot straight, or the torpedoes
to fly(?) straight, but a provision for unforseen environmentals
(cross-currents, shipping damage, etc.).

windage - offset left/right relative to target.
elevation - offset up/down relative to target.
'''


class interact_torpedo(sub):
    '''Executes interaction state for a task requiring launching torpedoes.
    
    Assumptions:
      - Torpedo launch is mapped to 1 or 2 joystick buttons (left and right
        launchers would need different windage caluclations applied, based on
        distance to target).
      - State is entered from a track_torpedo_target[TODO(travis): Update name]
        such that the desired target is aligned and centered.
    '''
    def __init__(self):
        smach.State.__init__(self, outcomes=['TORPEDO_SUCCESS',
                                             'TORPEDO_FAILURE'])
        #Const references to launcher trigger button maps.
        #TODO(travis):Actually map these to what mechanical/electrical use.
        self.LAUNCHER_LEFT = 'y'
        self.LAUNCHER_RIGHT = 'x'
        #Mapping for kentucky windages to apply to launchers.
        self.windage_map = {
                self.LAUNCHER_RIGHT: Kentucky(windage=0.0,elevation=0.0),
                self.LAUNCHER_LEFT: Kentucky(windage=0.0,elevation=0.0),
        }


    def execute(self, userdata):
        '''Executes the INTERACT_TORPEDO state's primariy action.'''
        rospy.loginfo('Executing state INTERACT_TORPEDO')
        try:
          tgt_range = self.get_range()
          launcher = self.next_launcher()
          self.apply_windage(launcher, tgt_range)
          self.launch(launcher)
          return 'Torpedo_Launched'
        except LaunchError as e:
            #Issues with launchers themselves - failure to ready, failure to
            #fire, etc.
            rospy.loginfo('INTERACT_TORPEDO - %s' % (e.message))
            return 'Torpedo_Failed'
        except Error as e:
            #Some other thing is broken, likely this very code.
            rospy.logwarn('INTERACT_TORPEDO - %s' % (e.message))
            return 'Torpedo_Failed'

    def launch(self, launcher):
        '''Launches a torpedo at the target.

        Args:
          launcher: string, id'd launcher, assumed armed and ready to fire.
        Raises:
          LaunchError: if there is an issue with the launcher(s) preventing
          torpedo launch.
        '''
        #TODO(travis):implement sending the joystick button press out.
        #TODO(travis):implement conditionals based on system health reporting.
        pass

    def next_launcher(self):
        '''Identifies the next launcher to fire, if more than one available.

        Depends upon relayed status messages and checks for launchers with
        READY status.

        Returns:
          string, joystick button mapping for next ready launcher.

        Raises:
          LauncherReadyError: if launcher status doesn't resolve to a READY
          state for at least one launcher in given timeout.
        '''
        #TODO(travis):implement conditionals based upon health reporting layer.
        return self.LAUNCHER_LEFT


    def apply_windage(self, launcher, tgt_range):
        '''Applies windage offset for given launcher.

        Args:
          launcher: string, reference to mapped windages.
          tgt_range: float, distance to target.
        '''
        #TODO(travis): implement method based upon range to apply KY windage.
        pass

    def get_range(self):
        '''Determines range to target based upon infering from sensor input.
       
        Uses values from the sensor package such as box_size, etc. to range
        the target.

        Returns:
          float, range to target.
        '''
        tgt_range = 0.0
        #TODO(travis): Determine ranging capabilities of sensor package.
        return tgt_range






