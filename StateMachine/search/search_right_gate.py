#!/usr/bin/env python

import rospy
import smach
import smach_ros

from  search_right import *

# define state Foo
class search_right_gate(search_right):
	pass

	def log(self):
		rospy.loginfo('Executing state SEARCH_RIGHT_GATE')