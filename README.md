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
- [Tf explanation](#Tf-explanation)
- [Behavior tree](#Behavior-tree)
- [BT NODES](#BT-NODES)
- [PID](#Pid)
- [Implements](#implements)
- [license](#license)

## Dependencies

To use this program, you will need to have the following packages installed:

- ROS2
- Darknet ROS (version 1.2 or later): This package provides a pre-trained convolutional neural network (CNN) for person detection using a camera.
- Groot: This is a graphical user interface (GUI) program that will allow you to manually control the robot. It is optional, but highly recommended.
- BehaviorTree.CPP: for the robot's actuaction we will use behaviour trees, by accessing the following link, you can clone the repository and follow the compilation steps: https://github.com/facontidavide/BehaviorTree.CPP
- ZMQLIB: as the behavior tree is external to ros, it needs an IOT communication middleware for the communication between nodes, this is why we use ZMQ.

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

colcon build

```

You can  install ZMQ following the nexts steps:
Snippet (install ZMQ):
``` bash
# search libzmq
apt-cache search zmq

# install
sudo apt-get install libzmq3-dev libboost-dev

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
Snippet (launch program):
``` bash

ros2 launch seekandcapture_cibernots actuation.launch.py 

```
-----------------------------------------------------------------------

## Tf explanation
The perception system implemented in the robot consists of publishing a transform with the position of the detected person.

For the detection of the person, darknet is used, which by means of different bounding boxes that show what is detected in the image, we can know if what is detected is a person or not.

With the person's bounding box, its center is found and its 3D point is obtained from the 2D image using the depth of the image by the pin-hole method of the camera.

With the 3D point a "detected_person" transform is created with respect to the fixed frame "odom", in this way we locate a person according to its transform.

Below is an image of the aforementioned transforms and a computer graph which graphically shows the different nodes that implement the created perception system.

### Transforms
<div align="center">
<img width=600px src="https://user-images.githubusercontent.com/90764494/228062043-d54209aa-46d8-424b-801e-4d76e9d219b2.png?raw=true" alt="explode"></a>
</div>

### Computer Graph
<div align="center">
<img width=600px src="https://user-images.githubusercontent.com/90764494/228062725-21386615-7479-492d-ac1e-7fba662bd6a2.png?raw=true" alt="explode"></a>
</div>

## Behavior tree
Remember that the most basic operation is a tick (a function call) that propagates to the children and returns 3 possibilities: SUCCES, FAILURE AND RUNNING.

The behaviour tree starts from a parent node **Repeat** with an input port *num_cycles* which is equal to 3 and indicates that the behaviour tree will be completed 3 times. 
It enters a **ReactiveFallback** (if it returns success the first child, it does not pass to the next one) This node has 3 child nodes:
  - **Inverter:** (decorator node that inverts what the child nodes return, if the child node returns SUCCESS, it changes it to FAILURE and vice versa.). This node has a **ReactiveFallback** child and this has two more childs:
       - **Detect Person**
       - **Search Person**      
  - **ReachedPerson**
  - **FollowPerson**
 
 The behaviour tree action nodes are explained below.

<div align="center">
<img width=600px src="https://user-images.githubusercontent.com/90764494/228063166-973cf105-81fa-4709-9135-5cbda9fba924.png?raw=true" alt="explode"></a>
</div>

## BT NODES

### Detect Person

If the transform is not found or too much time has passed between transforms, return FAILURE and tick the next node (Search Person). In other case, return SUCCESS

### Search Person

The tree enters this node when the DetectPerson node returns FAILURE (reactive sequence) and rotates until it finds a person. This node always return RUNNING.

### Reached Person

if it does not find the transform or if it finds it but the distance is not the requested one, it returns FAILURE. If the distance is the requested one, it publishes a sound and returns SUCCESS

### Follow Person

Always it returns RUNNING. The BT tick this node when detects a person but it's not reached yet. A PID is used to approach the person by adjusting the linear and angular velocities. Below we will see how the PID is implemented. 

## PID

## Implements

In addition to all the above-mentioned implementations, we created some more that we did not have time to test.

To fix a person we looked at the time stamp of a tf and the distance varied, if it exceeded certain thresholds, the person would have changed.
We also implemented a function to change the target, it consists of hitting the bumper, so that the robot is the one who must chase again. The robot, which will be listening on the topic of the bumper, when receiving an impact, will restart the bt. 
Finally, we added a subscriber to the lidar to be able to dodge local objects close to the robot, so that the robot would not crash and would have a safer navigation.


## license 
<a rel="license" href="https://www.apache.org/licenses/LICENSE-2.0"><img alt="Apache License" style="border-width:0" src="https://www.apache.org/img/asf-estd-1999-logo.jpg" /></a><br/>(Cibernots) </a><br/>This work is licensed under a <a rel="license" href="https://www.apache.org/licenses/LICENSE-2.0">Apache license 2.0
