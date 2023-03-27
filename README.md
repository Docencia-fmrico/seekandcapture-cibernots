# Seek_and_capture_cibernots

<h3 align="center">Seek and Capture </h3>

<div align="center">
<img width=100px src="https://img.shields.io/badge/status-finished-brightgreen" alt="explode"></a>
<img width=100px src="https://img.shields.io/badge/license-Apache-orange" alt="explode"></a>
</div>


## Table of Contents
- [Table of Contents](#table-of-contents)
- [How to execute the programs](#How-to-execute-the-programs)
- [Behavior tree](#Behavior_tree)
- [Tf explanation](#Tf_explanation)
- [Search Person](#Search_Person)
- [Detect Person](#Detect_Person)
- [Follow Person](#Follow_Person)
- [Reached Person](#Reached_Person)
- [Implements](#implements)
- [Tests](#tests)
- [Continuous integration](#Continuous-integration)
- [license](#license)

## Dependencies

To use this program, you will need to have the following packages installed:

    ROS2
    Darknet ROS (version 1.2 or later): This package provides a pre-trained convolutional neural network (CNN) for person detection using a camera.
    Groot: This is a graphical user interface (GUI) program that will allow you to manually control the robot. It is optional, but highly recommended.

You can install Darknet ROS by following the instructions in its GitHub repository:

Snippet (install Darknetros):
``` bash
cd ~/ros2_ws/src
git clone --recursive https://github.com/leggedrobotics/darknet_ros.git
cd darknet_ros
git checkout ros2
cd ../..
rosdep install --from-paths src --ignore-src --rosdistro foxy -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

```
You can also install Groot following the nexts steps:
Snippet (install Groot):
``` bash
cd ~/ros2_ws/src
git clone https://github.com/OUXT-Polaris/groot.git
cd groot
rosdep install -r --from-path .
colcon build

```

## How to execute the programs

First connect the base and the camera then :
-----------------------------------------------------------------------
Snippet (launch base):
``` bash
ros2 launch ir_robots kobuki.launch.py # Driver of the kobuki
```
-----------------------------------------------------------------------

-----------------------------------------------------------------------
Snippet (launch ""):
``` bash
```
-----------------------------------------------------------------------

## Behavior tree

## Tf explanation

## Search Person

## Detect Person

## Follow Person

## Reached Person

## Implements

## Tests

## Continuous integration


## license 
<a rel="license" href="https://www.apache.org/licenses/LICENSE-2.0"><img alt="Apache License" style="border-width:0" src="https://www.apache.org/img/asf-estd-1999-logo.jpg" /></a><br/>(Cibernots) </a><br/>This work is licensed under a <a rel="license" href="https://www.apache.org/licenses/LICENSE-2.0">Apache license 2.0
