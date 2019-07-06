#!/usr/bin/env python

from StateMachine.search.search_left import *

# define state search_left_pole
class search_left_pole(search_left):
	pass

	def log(self):
		rospy.loginfo('Executing state SEARCH_LEFT_POLE')