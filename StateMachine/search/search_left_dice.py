#!/usr/bin/env python

from StateMachine.search.search_left import *

# define state search_left_dice
class search_left_dice(search_left):
	pass

	def log(self):
		rospy.loginfo('Executing state SEARCH_LEFT_DICE')