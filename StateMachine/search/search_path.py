from StateMachine.sub import *

class Search_Path(Sub):
    def __init__(self):
        smach.State.__init__(self, outcomes= ['object_found', 'object_not found']

    def execute(self, userdata):
        self.init_state()
        gbl.state_heading = gbl.heading
        msg = self.init_joy_msg()
        msg.axes[const.AXES['frontback']] = 0.3

        while(1):
            self.publish_joy(msg)

            if self.get_box_of_class(gbl.detections_bottom, gbl.current_target):
                if self.search_frames_seen <= 2:
                    self.search_frames_seen += 1
                else:
                    return "object_found"

            else:
                self.search_frames_seen = 0
                return "object_not_found"
            rospy.sleep(const.SLEEP_TIME)
