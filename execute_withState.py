#!/usr/bin/env python

import rospy
import CreateStateMachine

def init_execute():
	rospy.loginfo("Starting up state machine...")
	CreateStateMachine.createStateMachine()

def main():
	init_execute()

if __name__ == '__main__':
	main()

