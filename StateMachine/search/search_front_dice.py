#!/usr/bin/env python

import rospy
import smach
import smach_ros

from search_front import *

# define state Foo
class search_front_dice(search_front):
	pass

	def log(self):
		rospy.loginfo('Executing state SEARCH_FRONT_DICE')
		