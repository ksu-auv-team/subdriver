#!/usr/bin/env python

from StateMachine.sub import *
from StateMachine import controllers

#TODO: clean up - this needs more work than I'm willing to do right now (while merging stuff)
#specifically implement a get_box_of_classes method (or decide to use a single buoy class),
# add a buoy class group, and reference PID() and msg correctly

# define state track_buoy
class Track_Buoy(Sub):
    """ This state starts with the sub having the buoy in sight.
    In this state, the sub will adjust its depth to match the 3 sided buoy and angle to face it. 
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['Lost_Buoy','Locked_Onto_Buoy'])

    def execute(self, userdata):
        rospy.loginfo('Executing state TRACK_BUOY')
        """ We will attempt to bump into the Drauger face of the buoy """
        self.init_state()
        msg = self.init_joy_msg()

        if not self.get_box_of_classes(detections, const.CLASS_GROUPS['buoy'])[0]:
            return "Lost_Buoy"


        # Line up with buoy
        # Since the buoy is spinning, there might be a problem lining up with it. Not sure what to do if that is the case
        # The below code assumes it is able to identify the spinning buoy the entire time.
        
        #TODO: implement this method and define this constant
        while self.get_center(self.get_box_of_classes(detections, const.CLASS_GROUPS['buoy'])[0]) != 0:
            self.matchBuoyDepth()
            self.matchBuoyLeftRight()
            self.moveCloseToBuoy()

        # At this point, the sub is stationary and facing the Buoy
        return 'Locked_Onto_Buoy'

    def matchBuoyDepth(self):
        rospy.loginfo("matchBuoyDepth: Adjusting depth")
        
        msg.axes[const.AXES['vertical']] = PID().update(self.get_center(gbl.detections[self.findBoxNumber()])[1])
        self.joy_pub.publish(msg)
        rospy.sleep(const.SLEEP_TIME)
        self.joy_pub.publish(msg)

    def matchBuoyLeftRight(self):
        rospy.loginfo("matchBuoyLeftRight: Adjusting left to right")
        msg.axes[const.AXES['leftright']] = PID().update(self.get_center(gbl.boxes[self.findBoxNumber()])[0])
        self.joy_pub.publish(msg)
        rospy.sleep(const.SLEEP_TIME)
        # Stop rotating
        msg.axes[const.AXES['leftright']] = 0
        self.joy_pub.publish(msg)
    
    def moveCloseToBuoy(self):
        rospy.loginfo("moveCloseToBuoy: Moving close to buoy")# While the image width of buoy is less than 0.75
        msg.axes[const.AXES['forward']] = 0.3
        rospy.sleep(const.SLEEP_TIME)
        msg.axes[const.AXES['forward']] = 0
        rospy.loginfo("Done adjusting distance")

        