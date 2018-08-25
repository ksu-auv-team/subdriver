#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy
from sensor_msgs.msg import FluidPressure
#from mavros_msgs.msg import VFR_HUD
import numpy as np
import time
import math
#import pymavlink
import random

import CreateStateMachine

if __name__ == '__main__':
	CreateStateMachine.createStateMachine()

