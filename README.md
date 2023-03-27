# Seek_and_capture_cibernots

<h3 align="center">Seek and Capture </h3>

<div align="center">
<img width=100px src="https://img.shields.io/badge/status-finished-brightgreen" alt="explode"></a>
<img width=100px src="https://img.shields.io/badge/license-Apache-orange" alt="explode"></a>
</div>


## Table of Contents
- [Table of Contents](#table-of-contents)
- [Dependencies](#Dependencies)
- [How to execute the programs](#How-to-execute-the-programs)
- [Behavior tree](#Behavior-tree)
- [Tf explanation](#Tf-explanation)
- [Search Person](#Search-Person)
- [Detect Person](#Detect-Person)
- [Follow Person](#Follow-Person)
- [Reached Person](#Reached-Person)
- [Implements](#implements)
- [Tests](#tests)
- [Continuous integration](#Continuous-integration)
- [license](#license)

## Dependencies

To use this program, you will need to have the following packages installed:

- ROS2
- Darknet ROS (version 1.2 or later): This package provides a pre-trained convolutional neural network (CNN) for person detection using a camera.
- Groot: This is a graphical user interface (GUI) program that will allow you to manually control the robot. It is optional, but highly recommended.

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

In addition to all the above-mentioned implementations, we created some more that we did not have time to test.

To fix a person we looked at the time stamp of a tf and the distance varied, if it exceeded certain thresholds, the person would have changed.
We also implemented a function to change the target, it consists of hitting the bumper, so that the robot is the one who must chase again. The robot, which will be listening on the topic of the bumper, when receiving an impact, will restart the bt. 
Finally, we added a subscriber to the lidar to be able to dodge local objects close to the robot, so that the robot would not crash and would have a safer navigation.

## Tests

## Continuous integration


## license 
<a rel="license" href="https://www.apache.org/licenses/LICENSE-2.0"><img alt="Apache License" style="border-width:0" src="https://www.apache.org/img/asf-estd-1999-logo.jpg" /></a><br/>(Cibernots) </a><br/>This work is licensed under a <a rel="license" href="https://www.apache.org/licenses/LICENSE-2.0">Apache license 2.0
