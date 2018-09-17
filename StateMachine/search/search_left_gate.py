#!/usr/bin/env python

from search_left import *

# define state Foo
class search_left_gate(search_left):
	pass

	def log(self):
		rospy.loginfo('Executing state SEARCH_LEFT_GATE')