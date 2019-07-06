#!/usr/bin/env python

from  StateMachine.search.search_right import *

# define state search_right_pole
class search_right_pole(search_right):
	pass

	def log(self):
		rospy.loginfo('Executing state SEARCH_RIGHT_POLE')