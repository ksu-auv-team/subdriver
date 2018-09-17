#!/usr/bin/env python

from StateMachine.sub import *

# define state Foo
class search_left(sub):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Found_Object','Not_Found_Object'])
	
	def log(self):
		pass

    def execute(self, userdata):
        self.log()
        return 'Not_Found_Object'

