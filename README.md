# Slither.io
A bio-inspired snake bot that follows the GAITS stored to perfom movements 
# Project 2


## Installing Dependencies
This program works on a device running Ubuntu 18.04 and ROS Melodic.

To install ROS Noetic in Ubuntu 18.04, follow the steps in this [link](http://wiki.ros.org/noetic/Installation/Ubuntu).

To install catkin, follow the installation steps in this [link](http://wiki.ros.org/catkin).

## How to build
Open a terminal window and run the following commands

```
mkdir -p ~/slither_ws/src
cd ~/slither_ws
catkin_make
source devel/setup.bash
cd src
git clone https://github.com/mrugesh1999/Slither.io
```

```
cd slither/scripts
chmod +x main.py

cd
cd slither_ws/
catkin_make
```

(Alternatively, copy the slither package into the 'slither_ws/src' directory and write 'catkin_make' in terminal)

## How to run Simulation
Open a terminal window and run the following commands

```
cd ~/slither_ws
source devel/setup.bash
roslaunch slither slither.launch

```
This launches the gazebo environment with out slither in it. 

Now, to control the slither following command is used in another terminal
```
cd ~/slither_ws
source devel/setup.bash
rosrun slither main.py
```

The following Message should appear

Control Your Toy!
---------------------------

k : Go Forward
l : Rotate Counter Clock Wise
s : Stop smoothly
CTRL-C to quit


## Maintainer ##
Mrugesh Shah (mrugesh.shah92@gmail.com)




