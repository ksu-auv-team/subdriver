#!/usr/bin/env python

from search_front import *

# define state search_front_dice
class search_front_buoy(search_front):
	def __init__(self):
		smach.State.__init__(self, outcomes = 'buoy_found', 'buoy_hit')

	def log(self):
		rospy.loginfo('Executing state SEARCH_FRONT_DICE')
		
