#!/usr/bin/env python

from StateMachine.search.search_recenter import *

# define state search_recenter_pole
class search_recenter_pole(search_recenter):
    pass


    def log(self):
        rospy.loginfo('Executing state SEARCH_RECENTER_POLE')