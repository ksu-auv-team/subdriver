#!/usr/bin/env python

from StateMachine.search.search_recenter import *

# define state search_recenter_gate
class search_recenter_gate(search_recenter):
	pass


	def log(self):
		rospy.loginfo('Executing state SEARCH_RECENTER_GATE')