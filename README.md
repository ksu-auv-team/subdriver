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
![networking_diagram](https://github.com/ksu-auv-team/subdriver/blob/master/media/Networking_Diagram.jpg "networking_diagram")

So, let me put this out there from the top: I hate networking. It only seems to get in the way of actual coding and it's incredibly confusing at times. That being said, it's a necessary evil.

Our system works by each device having a static IP address assigned by the router. We run a [DHCP server](https://www.linksys.com/us/support-article?articleNum=135673) on the router which you can set specific MAC addresses to have static IPs. So, for our purposes, the sub has a static IP, any new computer that we connect to it (personal or team laptops) typically get a new static IP. You can check the ip addresses of devices by going onto the router homepage by connecting to it and visiting `192.168.1.1` in your browser.

The system that you're using need to know where to send messages to and where they are comming from. That's where two special ROS environment variables come into play: `ROS_MASTER_URI` and `ROS_HOSTNAME`

If you want to read up more on environment variables, [be my guest](https://help.ubuntu.com/community/EnvironmentVariables), but we don't really need to know too much about them in order to use them. 

All you need to know is that `ROS_MASTER_URI` should be set to the ip address of where the master node is running (think where you ran roscore), and `ROS_MASTER_URI` should be the ip address of the computer that you're currently on. When we run the sub, we typically connect through a router where we run a dhcp server assigning static ip addresses we can use. But, if you want to set static ip addresses internally, that works too.

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

The last thing to think about when talking about networking is what happens when the sub is not connected to the router. Remember, the router is running a DHCP server which gives devices static ip addresses. So, when we disconnect from the router, that IP address goes away. However, in our .bashrc, we already told ROS to be looking for a specific IP address set by the router. So, before running totally untethered autonomy, you need set the following environment variables:

```bash
export ROS_HOSTNAME=localhost
export ROS_MASTER_URI=http://localhost:11311
```

This will tell ROS to not look for specific IP addresses, but look at the local computer.

## Running the State Machine

So you wanna run the state machine, huh?

Well, that's done through the `execute_withState.py` script. if you run a `python execute_withState.py -h` it will show a list of all the available commands that you can run.
Typically we want to pass in the value for whatever state machine we want to run, for example `python execute_withState.py -m BaseStateMachine` but by default the main state machine is run. 

By making your own state machines, you are able to test that the behavior of your new code behaves in the way that you expect it to. For example, if you're working on a new torpedo machine, you can wire up a state that lines you up with the torpedo board, then another state that fires a torpedo, then a final state that leaves the torpedo board. And all of this you can do in simulation, allowing you to check most of the functionality of your code without ever having to run it on the actual sub. 

## How to Write a State

TODO: Information on how to write a state for tutroial purposes

## How to Write a good StateMachine

TODO: Information on how to write a statemachine
