#!/usr/bin/env python

import rospy
import StateMachine.StateMachines.BaseStateMachine as base

def full_state_machine():
	rospy.loginfo("Running 'full_state_machine' ...")
	base.createStateMachine()

def main():
	full_state_machine()

if __name__ == '__main__':
	main()

