# Introduction to ROS: Beginner tutorials series
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://github.com/nr-parikh/beginner_tutorials/blob/master/LICENSE)

This repository showcases the tutorials on ROS and is a part of ENPM808X course at University of Maryland.

## Dependencies 
The dependencies of this repository are:

```
* Ubuntu 16.04
* ROS Kinetic Kame
```

Before proceedigng to install ROS, ensure that version of Ubuntu is 16.04. To install ROS follow the steps given below:

```
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
$ sudo apt-get update
$ sudo apt-get install ros-kinetic-desktop-full
```

After installation of ROS, the next step is to initialize and install its dependencies using following commands:

```
$ rosdep init
$ rosdep update 
```

The next step is to setup ROS environment which can be done using following commands:

```
$ echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```

Following the environment setup, next step is to create the workspace.

```
$ cd <path where workspace needs to be created>
$ mkdir -p <name of workspace>
$ cd <workspace>
<workspace>$ catkin_make
```

Just like the path of the ROS was sourced in *.bashrc* file, same needs to be done for the workspace by writing `source <path to workspace>/devel/setup.bash` at the end of the *.bashrc* file.
This avoids the need to source every time one needs to use workspace's packages.

## Building the code

The code can be built by cloning the repository and executing following steps:
```
<home>$ cd <workspace>/src
<workspace>/src$ git clone https://github.com/nr-parikh/beginner_tutorials.git
<workspace>/src$ cd ..
<workspace>$ catkin_make 
```

## Running the code

The code contains only has one node which publishes and subscribes to *chatter* topic. To run the code, ensure *rosmaster* is running. If it is not running, it can be started in a new terminal by executing `$ roscore`.
In another terminal, execute following command to run the node:
```
$ rosrun beginner_tutorials main_node
```

The output of the code is looks like follows:
```
[ INFO] [1509418147.690176981]: The incoming stream is: 
Welcome to ENPM808X, Fall 2017!
[ INFO] [1509418147.790819165]: The incoming stream is: 
Welcome to ENPM808X, Fall 2017!
[ INFO] [1509418147.890154578]: The incoming stream is: 
Welcome to ENPM808X, Fall 2017!
```
