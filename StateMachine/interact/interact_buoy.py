#!/usr/bin/env python
from gbl import Box_Type as BOX
from gbl import boxes
from StateMachine.sub import *
from StateMachine import controllers
from controllers import PID

# define state interact_buoy

# Needed information:
    # Distance from buoy
    # Rotation speed of buoy 
    # Current face on buoy
    # Order of faces (determined by which direction that the buoy is spinnning)

# Known Information:
    # Buoy Dimensions:  24 in. wide by 48 in. tall
    # Buoy will rotate 1-5 RPM
    # There is a two sided buoy with a Jiangshi on it. Do not target it
    # There is a three sided  buoy with Drauger, Vetalas, and Aswang on it.
        # We choose which one we will target and we will get 600 pts for hitting the one we chose
        # And 300 pts if we hit one of the other two.
    # Target: Since it is arbitrary, we will be targetting the Drauger
targetFace = BuoyFaces.Drauger

class interact_buoy(sub):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Clear_Of_Buoy'])

    def execute(self, userdata):
        """ We will attempt to bump into the Drauger face of the buoy """
        self.init_state()
        msg = self.init_joy_msg()
        box_num = -1
        for(i in range(0,len(boxes)):
            if(gbl.boxes[i][0]==BOX.buoy):# is a target
                box_num = i
        # At this point, the sub is stationary and facing the Buoy
        
        # Determine Ideal move speed
        
        distanceFromBuoy = findDistanceFromBuoy(boxes[box])
        
        buoyRotationSpeed = determineRotationSpeed(boxes[box]) # No idea what to do for this ******************************
        
        
        
        speedAndTime = calculateMovingSpeedAndTime(distanceFromBuoy)
        
        # Move towards
        rospy.loginfo("Moving forward")
        while rospy.get_time() > (gbl.run_start_time + 15):
            msg.axis[self.axis_dict['forward']]= 0.7
            self.joy_pub.publish(msg)
            rospy.sleep(gbl.sleep_time)

        gbl.current_target = None



        rospy.loginfo('Executing state INTERACT_BUOY')
        return 'Clear_Of_Buoy'
    
    def findDistanceFromBuoy(box):
        return -1
    def determineRotationSpeed(box):
        return -1
    
    def calculateMovingSpeedAndTime(distance, rotationSpeed, currentFace):
        """ Given the distance from the buoy and the rotation speed of the buoy"""
        return






