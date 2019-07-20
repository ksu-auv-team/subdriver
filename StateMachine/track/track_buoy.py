#!/usr/bin/env python

from StateMachine.sub import *

# define state track_buoy
class track_buoy(sub):
    """ This state starts with the sub having the buoy in sight.
    In this state, the sub will adjust its depth to match the 3 sided buoy and angle to face it. 
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['Lost_buoy','Locked_Onto_buoy'])

    def execute(self, userdata):
        rospy.loginfo('Executing state TRACK_BUOY')
        """ We will attempt to bump into the Drauger face of the buoy """
        self.init_state()
        msg = self.init_joy_msg()
        box_num = self.findBoxNumber()
        if(box_num == -1): # If the buoy was not
            return "Lost_buoy"
        # Line up with buoy
            # Since the buoy is spinning, there might be a problem lining up with it. Not sure what to do if that is the case
            # The below code assumes it is able to identify the spinning buoy the entire time.
        while((self.getCenter(gbl.boxes[self.findBoxNumber()])[1]!=0)or (self.getCenter(gbl.boxes[self.findBoxNumber()])[0]!=0) or ((gbl.boxes[self.findBoxNumber()][5] - gbl.boxes[self.findBoxNumber()][3]) < 0.75)):
            self.matchBuoyDepth()
            self.matchBuoyLeftRight()
            self.moveCloseToBuoy()

        # At this point, the sub is stationary and facing the Buoy
        return 'Locked_Onto_buoy'

    def findBoxNumber(self):
        for i in range (0, len(gbl.boxes)):
            if(gbl.gbl.boxes[i][0]==BOX.buoy):# is a target
                return i
        return -1

    def matchBuoyDepth(self):
        rospy.loginfo("matchBuoyDepth: Adjusting depth")
        
        msg.axis[self.axis_dict['vertical']] = PID().update(self.getCenter(gbl.boxes[self.findBoxNumber()])[1])
        self.joy_pub.publish(msg)
        rospy.sleep(gbl.const.const.SLEEP_TIME)
        # Stabilize
        msg.axis[self.axis_dict['vertical']] = self.depth_hold
        self.joy_pub.publish(msg)

    def matchBuoyLeftRight(self):
        rospy.loginfo("matchBuoyLeftRight: Adjusting left to right")
        msg.axis[self.axis_dict['leftright']] = PID().update(self.getCenter(gbl.boxes[self.findBoxNumber()])[0])
        self.joy_pub.publish(msg)
        rospy.sleep(gbl.const.const.SLEEP_TIME)
        # Stop rotating
        msg.axis[self.axis_dict['leftright']] = 0
        self.joy_pub.publish(msg)
    
    def moveCloseToBuoy(self):
        rospy.loginfo("moveCloseToBuoy: Moving close to buoy")# While the image width of buoy is less than 0.75
        msg.axis[self.axis_dict['forward']] = 0.3
        rospy.sleep(gbl.const.const.SLEEP_TIME)
        msg.axis[self.axis_dict['forward']] = 0
        rospy.loginfo("Done adjusting distance")

        