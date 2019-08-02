#/usr/bin/env python2

from StateMachine.sub import *

# define state interact_gate
class SpinToWin(Sub):
    def __init__(self):
        smach.State.__init__(self, outcomes=['through_gate', 'found_buoy'])

    def execute(self, userdata):
        self.init_state()
        gbl.state_heading = gbl.heading

        msg = self.init_joy_msg()
        msg.axes[const.AXES['frontback']] = 0.5     

        rospy.loginfo('Readying SPIN OF POWER FOR MAXIMUM STYLE')
 
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
        rospy.loginfo('Charging forward for four more seconds')
        while rospy.get_time() < (second_start_time + 4):
            msg.axes[const.AXES['frontback']] = 0.4
            self.publish(msg)
            rospy.sleep(const.SLEEP_TIME)

        curr_depth = self.get_depth()

        if gbl.debug:
            return 'found_buoy'

        #turn right 
        while self.get_depth() > curr_depth - 1 and abs(self.angle_diff(gbl.heading, gbl.state_heading + 15) > 4):
            msg = self.init_joy_msg()

            if self.get_depth() > curr_depth - 1:
                msg.axes[const.AXES['vertical']] = -0.2
            if abs(self.angle_diff(gbl.heading, gbl.state_heading + 15)) > 4:
                self.center_on_heading(gbl.state_heading + 15, msg)
            self.publish_joy(msg)

            if (self.get_box_of_class(gbl.detections_front, const.CLASSES['buoy_jiangshi'])):
                return 'found_buoy'
        
        return 'through_gate'


    def log(self):
        rospy.loginfo('Executing state SPIN_2_WIN')
