#!/usr/bin/env python2

#import roslib; roslib.load_manifest('smach_tutorials')
import rospy
import smach
from StateMachine.search import search

class Foo(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['outcome1','outcome2'])
		self.counter = 0

	def execute(self, userdata):
		#print('Executing state FOO')
		if self.counter < 3:
			self.counter += 1
			return 'outcome1'
		else:
			return 'outcome2'

class Bar(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['outcome1'])

	def execute(self, userdata):
		#print('Executing state BAR')
		return 'outcome1'


def main():
	rospy.init_node('smach_test')

	sm = smach.StateMachine(outcomes=['outcome4'])

	with sm:
		smach.StateMachine.add('FOO', Foo(),transitions={'outcome1':'BAR', 'outcome2':'outcome4'})

		smach.StateMachine.add('BAR', Bar(), transitions={'outcome1':'FOO'})

		outcome = sm.execute()


if __name__ == '__main__':
	main()
