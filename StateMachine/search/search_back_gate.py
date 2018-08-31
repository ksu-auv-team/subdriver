#!/usr/bin/env python

import rospy
import smach
import smach_ros

from  search_back import *

# define state Foo
class search_back_gate(search_back):
	pass


	def log(self):
		rospy.loginfo('Executing state SEARCH_BACK_GATE')