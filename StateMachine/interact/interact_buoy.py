#!/usr/bin/env python
from gbl import Box_Type as BOX
from gbl import boxes as boxes
from StateMachine.sub import *
from StateMachine import controllers
from controllers import PID
import math
from enum import Enum

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
        self.maxAcceleration = 20
    def execute(self, userdata):
        """ We will attempt to bump into the Drauger face of the buoy """
        rospy.loginfo('Executing state INTERACT_BUOY')
        # At this point, the sub is stationary and facing the Buoy
        self.init_state()
        msg = self.init_joy_msg()
        
        # Determine Ideal move speed
        
        
        
        buoyRotationSpeed = self.determineRotationSpeed() # No idea what to do for this ******************************
        rospy.loginfo("Found Rotation Speed.")

        distanceFromBuoy = self.findDistanceFromBuoy()
        rospy.loginfo("Found distance from Buoy.")
        
        accelerationAndTime = self.calculateAccelerationAndTime(distanceFromBuoy, buoyRotationSpeed)
        rospy.loginfo("Calculated movespeed and movement time.")
        acceleration = accelerationAndTime[0]
        time = accelerationAndTime[1]
        startTime = rospy.Time.now()
        # Move towards
        rospy.loginfo("Moving forward")
        while rospy.get_time() < startTime + time:
            msg.axis[self.axis_dict['forward']]= acceleration
            self.joy_pub.publish(msg)
            rospy.sleep(gbl.sleep_time)

        gbl.current_target = None
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
        firstFaceFoundTime = rospy.Time.now()
        
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
        """ Determine distance from buoy based on the height of the buoy,
        It can not be determined from the width since it is spinning. """
        cameraHeightConstant = -1 # Unknown as of now
        box = self.findBox()
        
        height = boxes[box][4] - boxes[box][6]

        return height * cameraHeightConstant
    
    def nextFace(self, face):
        if(face == BuoyFaces.Drauger):
            if(self.rotationOrder == BuoyRotationOrder.DAV):
                return BuoyFaces.Aswang
            else:
                return BuoyFaces.Vetalas
        elif(face == BuoyFaces.Aswang):
            if(self.rotationOrder == BuoyRotationOrder.DAV):
                return BuoyFaces.Vetalas
            else:
                return BuoyFaces.Drauger
        elif(face == BuoyFaces.Vetalas):
            if(self.rotationOrder == BuoyRotationOrder.DAV):
                return BuoyFaces.Drauger
            else:
                return BuoyFaces.Aswang
    def calculateAccelerationAndTime(self, distance, rotationSpeed):
        """ Given the distance from the buoy and the rotation speed of the buoy in RPM
        
        Stratagy:
            The buoy is divided into 3 sections and the current 
                face of the buoy can be measured as a function of time.
                Because each face will be shown for 1 third of the time we can represent this on a 
                unit circle with borders between sections drawn at 0, 2π/3, and 4π/3 Radians
                
            A Sinusodal function can be used to determine which third of the circle that
                the buoy is on as follows:
                
            First Third:
	            −1/2<Cos(t)<1
                0 <Sin(t)<=1
            Second Third:
	            −1≤Cos(t)<−1/2
                −√3/2<Sin(t)<√3/2
            Third Third:
	            −1/2<Cos(t)<1
                −1≤Sin(t)<0
            
            Calculating optimal Velocity:
                T = Period
                t = current time
                Use TCos(t) and TSin(t) to determine which face it is on.
                - Δx=1/2 at^2
                - a = 2Δx/(t^2)
                



        # Math for calculating period from rotation speed       # Physics
        revolutions/min * 1/60 = revolutions/second = w/(2Pi) = f
        T = 1/f ==> Period = 1/(rotationSpeed * 1/60) = 60/rotationSpeed
        """
        T = 60/rotationSpeed
        collisionWindow = T/3 # The time we will have to hit the buoy face once it shows.
        # Wait until the next face is visible
        startingFace = self.findFace()
        if(startingFace<0): # Currently inbetween faces
            if(self.buoyIsLost()):
                return (-1, -1)
        else: # Currently on a face
            currentFace = self.findFace()
            while(self.findFace() == currentFace):
                continue
            # Now it is in between faces
            if(self.buoyIsLost()):
                return (-1, -1)
        # Now it is the beginning of the nextFace
        face = self.findFace()
        targetThird = -1
        if(face==self.targetFace): # Target is the currentFace
            targetThird = 1
        elif(face == self.nextFace(self.targetFace)):
            targetThird = 3
        elif(self.nextFace(face) == self.targetFace):
            targetThird = 2
        else:
            return -1
        startTime = rospy.Time.now()
        acceleration = -1
        accelerationTime = -1
        for i in range(1, 50):
             if(self.getThirdAtTime(T, startTime, T*i/3) == targetThird and self.solveForAcceleration(distance, T*i/3)<self.maxAcceleration):
                 acceleration = self.solveForAcceleration(distance, T*i/3)
                 accelerationTime = T*i/3
                 return (acceleration, accelerationTime)
        return (-1, -1)
    def getThirdAtTime(self, period, startTime, currentTime):
        cos = period * math.cos(currentTime-startTime)
        sin = period * math.sin(currentTime-startTime)
        if(-1/2 < cos and cos < 1): # 1st or 3rd third
            if(sin>0):
                return 1
            else:
                return 3
        else:
            return 2
    def solveForAcceleration(self, distance, time):
        """ x = 1/2 at^2 ==> a = 2x/(t^2)"""
        return 2 * distance/(time ** 2)

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
    UnKnown = 3





