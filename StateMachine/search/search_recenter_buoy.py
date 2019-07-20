#!/usr/bin/env python

from StateMachine.search.search_recenter import *

# define state search_recenter_dice
class search_recenter_buoy(search_recenter):
	pass
	

	def log(self):
		rospy.loginfo('Executing state SEARCH_RECENTER_BUOY')