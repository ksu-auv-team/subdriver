#!/usr/bin/env python
from StateMachine.gbl import boxes
from StateMachine.sub import *
from StateMachine.controllers import PID
import math
from enum import Enum

# define state interact_buoy

# Known Information:
    # Buoy Dimensions:  24 in. wide by 48 in. tall
    # Buoy will rotate 1-5 RPM
    # There is a two sided buoy with a Jiangshi on it. Do not target it
    # There is a three sided  buoy with Drauger, Vetalas, and Aswang on it.
        # We choose which one we will target and we will get 600 pts for hitting the one we chose
        # And 300 pts if we hit one of the other two.
    # Target Face of buoy: Since it is arbitrary, we will be targetting the Drauger

# Calculated Information:
    # Distance from buoy
    # Rotation speed of buoy 
    # Current face on buoy
    # Order of faces (determined by which direction that the buoy is spinnning)

# Needed information:

#Python execute_withState
#logitec camera 720 x 1080p camera:
    # Logitec web cam C930e


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
        
        
        
        buoyRotationSpeed = self.determineRotationSpeed()
        rospy.loginfo("Found Rotation Speed.")

        # T = rotations/minute * 2pi rad/1 rotation * 60 seconds/1 minute
        period = buoyRotationSpeed * 2 * math.pi * 60

        # Move towards buoy
        rospy.loginfo("Preparing to move forward.")
        while findFace()!=BuoyFaces.Drauger:
            rospy.sleep(period/6)
            startTime=rospy.Time.now()
            rospy.loginfo("Moving forward for 10 seconds")
        while rospy.Time.now()<startTime+10000: # Move for 10 seconds
            msg.axis[self.axis_dict['forward']] = 1
            self.joy_pub.publish(msg)
            rospy.sleep(gbl.sleep_time)
        msg.axis[self.axis_dict['forward']] = 0
        rospy.loginfo("Done moving")
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
            rospy.loginfo("Buoy lost in def determineRotationSpeed(self) at time: ", rospy.Time.now())
            return -1
        startingFace = self.findFace()
        while(self.findFace() == startingFace):
            continue
        if(self.buoyIsLost()):
            rospy.loginfo("Buoy lost in def determineRotationSpeed(self) at time: ", rospy.Time.now())
            return -1
        firstFaceFoundTime = rospy.Time.now()
        
        firstFace = self.findFace()
        # Find second face
        while(self.findFace() == firstFace):
            continue
        if(self.buoyIsLost()):
            rospy.loginfo("Buoy lost in def determineRotationSpeed(self) at time: ", rospy.Time.now())
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

    def getThirdAtTime(self, period, startTime, currentTime):
        # T = 2pi/w
        # Formula for a sinusodal function: y = ASin(wt + phaseShift)
        w = 2*math.pi/period
        x =  math.cos(w(currentTime-startTime))
        y = math.sin(w(currentTime-startTime))
        if(-1/2 < x and x < 1): # 1st or 3rd third
            if(y>0):
                return 1
            else:
                return 3
        else: # 2nd Third
            return 2


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





