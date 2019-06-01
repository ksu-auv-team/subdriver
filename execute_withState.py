#!/usr/bin/env python2

import rospy
import StateMachine.machines.BaseStateMachine as base
import argparse

parser = argparse.ArgumentParser(description="execute a state machine for the submarine")
parser.add_argument('-m', '--machine', default = "full_state_machine", help="the name of the state machine to execute (default: %(default)s)")
args = parser.parse_args()

def hello_world():
	print("Hi! I'm useless!")

states = {
	'full_state_machine': base.createStateMachine,
	'test_machine': hello_world,
	'this_is_filler': 2,
	'to_show_the_concept': 3,
}

def main():
	rospy.loginfo("Running {}".format(args.machine))
	try:
		states[args.machine]()
	except KeyError:
		rospy.logfatal("Error: state machine name not recognized")

if __name__ == '__main__':
	main()

