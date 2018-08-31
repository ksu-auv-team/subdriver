#!/usr/bin/env python

import rospy
import smach
import smach_ros

from search_left import *

# define state Foo
class search_left_gate(search_left):
	pass

	def log(self):
		rospy.loginfo('Executing state SEARCH_LEFT_GATE')