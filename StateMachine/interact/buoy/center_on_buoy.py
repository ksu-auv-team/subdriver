#!/usr/bin/env python
from StateMachine.gbl import *
from StateMachine.const import *
from StateMachine.sub import *
from StateMachine.controllers import PID
import math
from enum import Enum

'''
Defines substate of Sub that will center us on a buoy.
It's designed to use current_target so we can use it on multiple faces.
Expects to start near enough to the buoys that we can see them, but not, like, super close?
Ideally we'd be able to see both.

Will get the sub to the center of the face first
'''


class Center_On_Buoy(Sub):
    def __init__(self):
        smach.State.__init__(self, outcomes=['centered_first_buoy', 'centered_second_buoy', 'lost_buoy'])

    def execute(self, userdata):
        rospy.loginfo('Executing state CENTER_ON_BUOY')
        self.init_state()
        msg = self.init_joy_msg()     
        
        buoyRotationSpeed = self.determineRotationSpeed()
        rospy.loginfo("Found Rotation Speed.")

        # T = rotations/minute * 2pi rad/1 rotation * 60 seconds/1 minute
        period = buoyRotationSpeed * 2 * math.pi * 60

        # Move towards buoy
        rospy.loginfo("Preparing to move forward.")
        while self.findFace() != BuoyFaces.draugr:
            rospy.sleep(period/6)
            startTime=rospy.Time.now()
            rospy.loginfo("Moving forward for 10 seconds")
        
        while rospy.Time.now() < startTime + 10000: # Move for 10 seconds
            msg.axes[const.AXES['forward']] = 1
            self.publish(msg)
            rospy.sleep(const.SLEEP_TIME)
        
        msg.axes[const.AXES['forward']] = 0
        rospy.loginfo("Done moving")
        gbl.current_target = None
        return 'clear_of_buoy'

    def determineRotationSpeed(self):
        """ returns rotation speed in RPM,
        or Returns -1 if unable to find order, but that doesn't necessarily mean that the 
        buoy is lost."""
        # Find first face
        # Skip the first face found, because it could be found halfway through the time spent on that face.
        if(self.buoyIsLost()):
            rospy.loginfo("Buoy lost in def determineRotationSpeed(self) at time: ", rospy.Time.now())
            return -1
        
        
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
        
        if(firstFace == secondFace):
            return -1
        # Determine Order by waiting for a face to appear, then waiting until the next face appears
        
        if(firstFace == BuoyFaces.draugr): #draugr
            
            if(secondFace == BuoyFaces.aswang):
                self.rotationOrder = BuoyRotationOrder.DAV
                return (secondFaceFoundTime - firstFaceFoundTime) * 3 * 60
        
            elif(secondFace == BuoyFaces.vetalas):
                self.rotationOrder = BuoyRotationOrder.VAD
                return (secondFaceFoundTime - firstFaceFoundTime) * 3 * 60
        
            else:
                return -1
        
        elif(firstFace == BuoyFaces.aswang): #aswang
            
            if(secondFace == BuoyFaces.draugr):
                self.rotationOrder = BuoyRotationOrder.VAD
                return (secondFaceFoundTime - firstFaceFoundTime) * 3 * 60
            
            elif(secondFace == BuoyFaces.vetalas):
                self.rotationOrder = BuoyRotationOrder.DAV
                return (secondFaceFoundTime - firstFaceFoundTime) * 3 * 60
            
            else:
                return -1
        
        elif(firstFace == BuoyFaces.vetalas): #vetalas
            if(secondFace == BuoyFaces.aswang):
                self.rotationOrder = BuoyRotationOrder.VAD
                return (secondFaceFoundTime - firstFaceFoundTime) * 3 * 60
            elif(secondFace == BuoyFaces.draugr):
                self.rotationOrder = BuoyRotationOrder.DAV
                return (secondFaceFoundTime - firstFaceFoundTime) * 3 * 60
            else:
                return -1
        else:
            return -1

    def buoyIsLost(self):
        # Waits a while until it sees the buoy. Returns true if buoy is lost.
        """ Wait up to 30 seconds to see if the buoy any of the monsters on the 3 sided buoy are 
        found. If none are found, then the buoy is assumed to be lost. """
        start = rospy.Time.now()
        while(rospy.Time.now()<start+30):
            if(self.findFace()>=0):
                return False
        return True
           
    def nextFace(self, face):
        # Given a buoy face, method will return the next face in the order
        if(face == BuoyFaces.draugr):
            if(self.rotationOrder == BuoyRotationOrder.DAV):
                return BuoyFaces.aswang
            else:
                return BuoyFaces.vetalas
        
        elif(face == BuoyFaces.aswang):
            if(self.rotationOrder == BuoyRotationOrder.DAV):
                return BuoyFaces.vetalas
            else:
                return BuoyFaces.draugr
        
        elif(face == BuoyFaces.vetalas):
            if(self.rotationOrder == BuoyRotationOrder.DAV):
                return BuoyFaces.draugr
            else:
                return BuoyFaces.aswang

    def getThirdAtTime(self, period, startTime, currentTime):
        # Finds out what face will be showing at a certain time
        # The reason for a startime parameter is because the state does not start at time 0.
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
    draugr = 0
    aswang = 1
    vetalas = 2

class BuoyRotationOrder(Enum):
    """Enumeration for if the rotation order is:
     draugr, aswang, vetalas (DAV)
     or vetalas, aswang, draugr (VAD)
    """
    DAV = 0
    VAD = 1
    Unknown = 3





