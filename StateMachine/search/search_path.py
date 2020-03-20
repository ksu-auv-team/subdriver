from StateMachine.edu import *

class Search_path(sub):
  def_init_(self):
    smach.State._init(self, outcomes= (['object_found', object_not found])
    
    def execute(self, userdata): 
    self.__init__state()
    gbl.state_heading = gbl.heading
    msg = self._init_joy_msg()
    msg.axes[const.AXES['frontback']] = 0.3
    

start 

while(1):
            self.joy_pub.publish(msg)

            if self.get_box_of_class(gbl.detections_bottom, gbl.current_target):
                if self.search_frames_seen <= 2:
                    self.search_frames_seen += 1
                    
 else:
                self.search_frames_seen = 0
                return "object_not_found'

            rospy.sleep(const.SLEEP_TIME)    
