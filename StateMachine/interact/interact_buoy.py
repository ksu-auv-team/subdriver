#!/usr/bin/env python
from gbl import Box_Type as BOX
from gbl import boxes
from StateMachine.sub import *
from StateMachine import controllers
from controllers import PID

# define state interact_dice
class interact_buoy(sub):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Clear_Of_Buoy'])

    def execute(self, userdata):
        self.init_state()
        msg = self.init_joy_msg()
        box = None
        for(i in range(0,len(gbl.boxes)):
            if(gbl.boxes[i][0]==BOX.buoy): # is a target
                box_num = i


        rospy.loginfo("Adjusting depth")
        while(self.getCenter(boxes[box])[1]!=0):
            msg.axis[self.axis_dict['vertical']] = PID().update(self.getCenter(boxes[box])[1])
            self.joy_pub.publish(msg)
            rospy.sleep(gbl.sleep_time)
        # Stabilize
        msg.axis[self.axis_dict['vertical']] = self.depth_hold
        self.joy_pub.publish(msg)

        rospy.loginfo("Adjusting left to right")
        while(self.getCenter(boxes[box])[0]!=0):
            msg.axis[self.axis_dict['leftright']] = PID().update(self.getCenter(boxes[box])[0])
            self.joy_pub.publish(msg)
            rospy.sleep(gbl.sleep_time)
        # Stop rotating
        msg.axis[self.axis_dict['leftright']] = 0
        self.joy_pub.publish(msg)

        rospy.loginfo("Moving forward")
        while rospy.get_time() > (gbl.run_start_time + 15):
            msg.axis[self.axis_dict['forward']]= 0.7
            self.joy_pub.publish(msg)
            rospy.sleep(gbl.sleep_time)

        gbl.current_target = None



        rospy.loginfo('Executing state INTERACT_BUOY')
        return 'Clear_Of_Buoy'
