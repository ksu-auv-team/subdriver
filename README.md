# State Machine Executor

TODO: add in environment setup suggestions; where to get/install rospy for local development, and other library deps. maybe even an env setup script (in a containerized python env, for consistency in dev experience).

TODO: Talk about ROS_MASTER_URI and networking stuff

TODO: Show example running stuff

## Install
```bash
sudo apt-get install libxml2-dev libxslt-dev python-dev
sudo pip2 install -U pymavlink
```

## Catkin_Ws
```bash
cd /path/to/subdriver
echo "source $(pwd)/catkin_ws/devel/setup.bash" >> ~/.bashrc
cd catkin_ws
catkin_make
source ~/.bashrc
```
