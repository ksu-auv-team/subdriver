# State Machine Executor

TODO: add in environment setup suggestions; where to get/install rospy for local development, and other library deps. maybe even an env setup script (in a containerized python env, for consistency in dev experience).

TODO: Talk about ROS_MASTER_URI and networking stuff

TODO: Show example running stuff

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



