#/usr/bin/env python2

from StateMachine.sub import *

# define state interact_gate
class SpinToWin(Sub):
    def __init__(self):
        smach.State.__init__(self, outcomes=['through_gate'])

    def execute(self, userdata):
        self.init_state()
        gbl.state_heading = gbl.heading

        msg = self.init_joy_msg()
        msg.axes[const.AXES['frontback']] = 0.5     

        rospy.loginfo('Charging forward for FORWARD_DIST seconds')
 
        while rospy.get_time() < (self.current_state_start_time + 3):
            self.publish(msg)
            rospy.sleep(const.SLEEP_TIME)

        rospy.loginfo('720noscope')
        degrees_spun = 0

        if not gbl.debug:
            last_heading = gbl.state_heading
            while (degrees_spun < 585):
                msg.axes[const.AXES['frontback']] = 0   
                msg.axes[const.AXES['rotate']] = -0.3
                degrees_spun += self.angle_diff(last_heading, gbl.heading)
                last_heading = gbl.heading
                self.publish_joy(msg)
            
        msg = self.init_joy_msg()
        heading_held_time = 0.0

        if not gbl.debug:
            while (abs(self.angle_diff(gbl.heading, gbl.state_heading)) > 3):
                msg.axes[const.AXES['frontback']] = 0
                msg = self.center_on_heading(gbl.state_heading, msg)
                self.publish_joy(msg)
        
        msg = self.init_joy_msg()
        heading_held_time = rospy.get_time()

        if not gbl.debug:
            while rospy.get_time() - heading_held_time < 1:
                if abs(self.angle_diff(gbl.heading, gbl.state_heading)) > 3:
                    heading_held_time = rospy.get_time()
                msg = self.center_on_heading(gbl.state_heading, msg, min_thrust=0.05, max_thrust=0.2)
                self.publish_joy(msg)
            
        msg = self.init_joy_msg()

        second_start_time = rospy.get_time()
        rospy.loginfo('Charging forward for three more seconds')
        while rospy.get_time() < (second_start_time + 3):
            msg.axes[const.AXES['frontback']] = 0.15
            self.publish(msg)
            rospy.sleep(const.SLEEP_TIME)

        return 'through_gate'


    def log(self):
        rospy.loginfo('Executing state SPIN_2_WIN')
