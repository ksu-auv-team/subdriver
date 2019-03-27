#!/usr/bin/env python

import rospy
import StateMachine.StateMachines.CreateStateMachine as sm

def full_state_machine():
	rospy.loginfo("Running 'full_state_machine' ...")
	sm.createStateMachine()

def main():
	full_state_machine()

if __name__ == '__main__':
	main()

