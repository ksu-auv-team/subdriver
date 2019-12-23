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

All you need to know is that `ROS_MASTER_URI` should be set to the ip address of where the master node is running (think where you ran roscore), and `ROS_HOSTNAME` should be the ip address of the computer that you're currently on. When we run the sub, we typically connect through a router where we run a dhcp server assigning static ip addresses we can use. But, if you want to set static ip addresses internally, that works too.

When I'm running things from my computer, I typically add some lines in my .bashrc to easily switch between running ROS locally and running it on the sub. 

This one is for running on your local machine.

```bash
export ROS_HOSTNAME=localhost
export ROS_MASTER_URI=http://localhost:11311
```

This one is for hooking up to the sub's roscore

```bash
export ROS_HOSTNAME=<your-static-ip-address>
export ROS_MASTER_URI=http://<ip-address-of-sub>:11311
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

We use a ROS package called [SMACH](http://wiki.ros.org/smach) that organizes our state machine into smaller pieces. This helps us abstract the individual pieces of the way we run the sub, meaning we can re-use pieces previously coded super easily. All a new user has to do is re-wire a new state machine (described below) from a whole bunch of little 'states' as described here.

Firstly, let's get some logistics of the anatomy of a state out of the way. Every single one of our states inherit from a super class called `sub.py`. This super class holds lots of useful methods and bits of code that make it much easier to develop, [take a look at it](https://github.com/ksu-auv-team/subdriver/blob/master/StateMachine/sub.py) before making a new state to be familliar with what's available to you.

Next, each one of these states has *hard* requirements, and some *soft* requirements. The two hard requirements are `__init__(self)` and `execute(self, userdata)`. These are two methods that SMACH requires of us to override when we make a new state. While fairly self-explainitory, init happens in the initialization of the state, while execute is the actual execution of the state. `__init__` requires you to define all the possible outcomes of the state, while `execute` requires that you return one of those defined outcomes. Alos, **an important note** `execute` only happens **once** so, if you want it to loop through some behavior, you need to have a loop inside of `execute`.

Lastly, some of the soft requirements are just things that you should be doing in most states to initialize them for general helpfulness. The first of which is running `self.init_state()` in the beginning of your `execute` which logs some things internally. You can check out the `sub.py` class to see the specifics of what it's doing. Additionally, it's helpful to log some info to the terminal using `rospy.loginfo('Your message here!')`

Now that that's out of the way, writing a new state is as easy as 1, 2, 3:

1. Decide where you want your state to start. What assumptions are you going to make that your system in in when starting?
2. Decide where your system can transition to next. What are all the possible end conditions of your state?
3. Implement the logistics of the your state, and make sure that you adhere to the above two rules.

#### 1. Where to start  
This first step is more of a theoretical step as opposed to a hard requirement. This is something that should direct your future writing.

Unlike with the future transitions, SMACH does not require you to program in where your current state needs to start at. You should try and think of your state as a stand-alone idea with as little reliance on outside information as possible. Also, try and make your state as small and distinct as possible without getting ridiculous. The smaller the state, the easier it is to re-use it for future state machines. 

Commenting at the beginning of the state is extremely helpful also. You should be letting future users know where they are and where they can slot this state into a larger picture as well as what exactly this state is attempting to do. 

#### 2. Where to go  
This step is a hard requrement of SMACH. As mentioned before, in your `__init__` you need to tell the state what are the possible outcomes of this state. For example, most of the time the first state we start in is called `start.py` and it has two outcomes in the `__init__` as follows:

```python
def __init__(self):
    smach.State.__init__(self, outcomes=['not_found_gate', 'found_gate'])
```

This tells SMACH that we have two possible outcomes, 'not_found_gate', and 'found_gate'. These possible outcomes will influience how we wire up state machines in the future depending on what our outcomes are. 

After you have defined what these outcomes are in the `__init__`, you will need to return one of these in your `execute`. That's how SMACH works, you return the outcome that actually happened and the state machine uses that information to transition to the next position in the system. For example, again from `start.py`:

```python
# Control loop
while(1):
   self.publish_joy(curr_msg)

    if(rospy.get_time() - self.current_state_start_time) > 2:    
        if self.get_box_of_class(gbl.detections_front, const.CLASSES['start_gate']):
            return 'found_gate'
        elif (rospy.get_time() - self.current_state_start_time) > 6:
            return 'not_found_gate'

    rospy.sleep(const.SLEEP_TIME)
```

What this section of code is doing, is it's trying to decide if we have found the start_gate or not between the first 2 to 6 seconds of running. If we have found the gate, we return 'found_gate' as a string, but if we have not found it, we return 'not_found_gate'. 

#### 3. Implementation  
When it comes down to implementation, it's more art than science, but here's some very practical tips for you to get started with.

1. Follow the above steps for all the logistical stuff: Inherit from sub, setup possible outcomes in `__init__`, run `self.init_state()` in execute, you will probably need a loop to run your code in inside of `execute`, use `rospy.loginfo()` as much as is useful, return whatever outcome is applicable. 
2. Try not to use system sleep delays or `rospy.sleep()`, but use the 'wait without delay' types of programming. If that doesn't make sense, take a look at [this arduino LED blink](https://www.arduino.cc/en/tutorial/BlinkWithoutDelay) as an example, or the above python example.
3. Utilize the `sub.py` class as much as possible, it's got a lot of useful methods in it. 
4. Take a look at `start.py` as a good example of how to write your first state. It shows you how to fill up a joystick message and write a control loop.
5. For joystick messages, they are in the range of -1 to +1 for all their axes. The only tricky thing is positive X and Y is the **TOP LEFT** of each direction, and negative X and Y is **BOTTOM RIGHT** which is against what I would think. This means that if you want the sub to strafe right at max speed, you would give the 'strafe' axes a value of '-1'. If you wanted to go down at max speed, you would give the 'vertical' axes a value of '-1'.
6. When publishing joystick messages, use `self.publish_joy` from the `sub.py` class. It makes global edits easier to do. 
7. We can run the autonomy with the `--debug` flag which just makes `gbl.debug` true. Use this to your advantage when working on your code, incorporate parts of code to skip over when running debug mode. Any infinite loops should have an end condition for a `--debug` option. Again, check `start.py` for an example. 
8. We have 2 special files, `gbl.py` and `const.py`. `gbl.py` is data that is designed to be changing constantly and only ever read from. Think 'current depth' or 'current heading', the most up to date information on this data. `const.py` is designed to hold constants that will only change if there is actual hardware changes to the sub, or we add new physical functionality. These are things like the controller buttons/axes for joysticks.

## How to Write a good StateMachine

TODO: Information on how to write a statemachine
