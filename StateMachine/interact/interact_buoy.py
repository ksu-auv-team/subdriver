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



class interact_buoy(sub):
    
    

    def __init__(self):
        smach.State.__init__(self, outcomes=['Lost_Buoy', 'Clear_Of_Buoy'])
        self.rotationOrder = -1
        self.targetFace = BuoyFaces.Drauger
    def execute(self, userdata):
        """ We will attempt to bump into the Drauger face of the buoy """
        # At this point, the sub is stationary and facing the Buoy
        self.init_state()
        msg = self.init_joy_msg()
        
        # Determine Ideal move speed
        
        
        
        buoyRotationSpeed = self.determineRotationSpeed(order) # No idea what to do for this ******************************
        rospy.loginfo("Found Rotation Speed.")

        distanceFromBuoy = self.findDistanceFromBuoy()
        rospy.loginfo("Found distance from Buoy.")
        
        speedAndTime = calculateMovingSpeedAndTime(distanceFromBuoy, order, face)
        rospy.loginfo("Calculated movespeed and movement time.")
        
        # Move towards
        rospy.loginfo("Moving forward")
        while rospy.get_time() > (gbl.run_start_time + 15):
            msg.axis[self.axis_dict['forward']]= 0.7
            self.joy_pub.publish(msg)
            rospy.sleep(gbl.sleep_time)

        gbl.current_target = None



        rospy.loginfo('Executing state INTERACT_BUOY')
        return 'Clear_Of_Buoy'
    def findBox(self):
        for i in range(0, len(boxes)):
            if(boxes[i][1] == BuoyFaces.Drauger or boxes[i][1] == BuoyFaces.Aswang or boxes[i][1] == BuoyFaces.Vetalas):
                return i[1]
        return -1 # Face was not found
    def findFace(self):
        box = self.findBox()
        if(box<0):
            return -1
        return boxes[box][1]
    def determineRotationSpeed(self):
        """ returns rotation speed in RPM,
        or Returns -1 if unable to find order, but that doesn't necessarily mean that the 
        buoy is lost."""
        # Find first face
        # Skip the first face found, because it could be found halfway through the time spent on that face.
        if(self.buoyIsLost()):
            return -1
        startingFace = self.findFace()
        while(self.findFace() == startingFace):
            continue
        if(self.buoyIsLost()):
            return -1
        firstFaceFoundTime = rospy.Time.now)()
        
        firstFace = self.findFace()
        # Find second face
        while(self.findFace() == firstFace):
            continue
        if(self.buoyIsLost()):
            return -1
        secondFaceFoundTime = rospy.Time.now()
        secondFace = self.findFace()
        if(firstFace == secondFace):
            return -1
        # Determine Order
        if(firstFace == BuoyFaces.Drauger):
            if(secondFace == BuoyFaces.Aswang):
                self.rotationOrder = BuoyRotationOrder.DAV
                return (secondFaceFoundTime - firstFaceFoundTime) * 3 * 60
            elif(secondFace == BuoyFaces.Vetalas):
                self.rotationOrder = BuoyRotationOrder.VAD
                return (secondFaceFoundTime - firstFaceFoundTime) * 3 * 60
            else:
                return -1
        elif(firstFace == BuoyFaces.Aswang):
            if(secondFace == BuoyFaces.Drauger):
                self.rotationOrder = BuoyRotationOrder.VAD
                return (secondFaceFoundTime - firstFaceFoundTime) * 3 * 60
            elif(secondFace == BuoyFaces.Vetalas):
                self.rotationOrder = BuoyRotationOrder.DAV
                return (secondFaceFoundTime - firstFaceFoundTime) * 3 * 60
            else:
                return -1
        elif(firstFace == BuoyFaces.Vetalas):
            if(secondFace == BuoyFaces.Aswang):
                self.rotationOrder = BuoyRotationOrder.VAD
                return (secondFaceFoundTime - firstFaceFoundTime) * 3 * 60
            elif(secondFace == BuoyFaces.Drauger):
                self.rotationOrder = BuoyRotationOrder.DAV
                return (secondFaceFoundTime - firstFaceFoundTime) * 3 * 60
            else:
                return -1
        else:
            return -1

    def buoyIsLost(self):
        """ Wait up to 30 seconds to see if the buoy any of the monsters on the 3 sided buoy are 
        found. If none are found, then the buoy is assumed to be lost. """
        start = rospy.Time.now()
        while(rospy.Time.now()<start+30):
            if(self.findFace()>=0):
                return False
        return True
    def findDistanceFromBuoy(self):
        return -1
    
    
    def calculateMovingSpeedAndTime(distance, rotationSpeed, currentFace):
        """ Given the distance from the buoy and the rotation speed of the buoy"""
        return

class BuoyFaces(Enum): # Numbers will need to be changed to a proper class_id
    Drauger = 0
    Aswang = 1
    Vetalas = 2
class BuoyRotationOrder(Enum):
    """Enumeration for if the rotation order is:
     Drauger, Aswang, Vetalas,  , (DAV)
     or Vetalas, Aswang, Drauger (VAD)
    """
    DAV = 0
    VAD = 1





