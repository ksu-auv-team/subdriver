#!/usr/bin/env python

from StateMachine.search.search_front import *

# define state search_front_pole
class Search_Front_Pole(Search_Front):
	pass

	def log(self):
		rospy.loginfo('Executing state SEARCH_FRONT_POLE')