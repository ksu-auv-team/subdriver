#!/usr/bin/env python

from StateMachine.search.search_front import *

# define state search_front_gate
class search_front_gate(search_front):
	pass

	def log(self):
		rospy.loginfo('Executing state SEARCH_FRONT_GATE')