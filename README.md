# State Machine Executor

## Install
The most up to date information will be in the super repo for this repo [sub-utilities](https://github.com/ksu-auv-team/sub_utilities). You should check there under the **subdriver** section for instalation notes.


## catkin\_ws

We have some custom ROS messages setup for our system, so we need to install them inside of a catkin\_ws. To do that, follow the instructions below.

```bash
cd /path/to/subdriver
echo "source $(pwd)/catkin_ws/devel/setup.bash" >> ~/.bashrc
cd catkin_ws
catkin_make
source ~/.bashrc
```

## Networking and You
So, let me put this out there from the top: I hate networking. It only seems to get in the way of actual coding and it's incredibly confusing at time. 

That being said, it's a necessary evil. The system that you're using need to know where to send messages to and where they are comming from. That's where two special ROS environment variables come into play: `ROS_MASTER_URI` and `ROS_HOSTNAME`

If you want to read up more on environment variables, [be my guest](https://help.ubuntu.com/community/EnvironmentVariables), but we don't really need to know too much about them in order to use them. 

All you need to know is that `ROS_MASTER_URI` should be set to the ip address of where the master node is running (think where you ran roscore), and `ROS_MASTER_URI` should be the ip address of the computer that you're currently on. When we run the sub, we typically connect through a router where we run a dhcp server assigning static ip addresses we can use.

When I'm running things from my computer, I typically add some lines in my .bashrc to easily switch between running ROS locally and running it on the sub. 

This one is for running on your local machine.

```bash
export ROS_HOSTNAME=localhost
export ROS_MASTER_URI=http://localhost:11311
```

This one is for hooking up to the sub's roscore

```bash
export ROS_HOSTNAME=<your-static-ip-address>
export ROS_MASTER_URI=http://<ip-address-of-sub>/:11311
```

I tend to put both of these things in my `.bashrc` and comment out one of them. After commenting out one, remember to `source ~/.bashrc` in each open terminal so that change takes effect.

## Running the State Machine

So you wanna run the state machine, huh?

Well, that's done through the `execute_withState.py` script. if you run a `python execute_withState.py -h` it will show a list of all the available commands that you can run.
Typically we want to pass in the value for whatever state machine we want to run, for example `python execute_withState.py -m BaseStateMachine` but by default the main state machine is run. 

By making your own state machines, you are able to test that the behavior of your new code behaves in the way that you expect it to. For example, if you're working on a new torpedo machine, you can wire up a state that lines you up with the torpedo board, then another state that fires a torpedo, then a final state that leaves the torpedo board. And all of this you can do in simulation, allowing you to check most of the functionality of your code without ever having to run it on the actual sub. 

## How to Write a State

TODO: Information on how to write a state for tutroial purposes

## How to Write a StateMachine

TODO: Information on how to write a statemachine
