# Seekandcapture_cibernots

<h3 align="center">Seek and Capture </h3>

<div align="center">
<img width=100px src="https://img.shields.io/badge/status-finished-brightgreen" alt="explode"></a>
<img width=100px src="https://img.shields.io/badge/license-Apache-orange" alt="explode"></a>
</div>


## Table of Contents
- [Table of Contents](#table-of-contents)
- [How to execute the programs](#How-to-execute-the-programs)
- [Search Person](#Search_Person)
- [Detect Person](#Detect_Person)
- [Follow Person](#Follow_Person)
- [Parameters](#parameters)
- [Implements](#implements)
- [Tests](#tests)
- [Continuous integration](#Continuous-integration)
- [license](#license)


## How to execute the programs

First connect the base and the lidar then :
-----------------------------------------------------------------------
Snippet (launch base):
``` bash
ros2 launch ir_robots kobuki.launch.py # Driver of the kobuki
```
-----------------------------------------------------------------------

-----------------------------------------------------------------------
Snippet (launch avoid_obstacle_cibernots):
``` bash
ros2 launch avoid_obstacle_cibernots avoid_obstacle.launch.py  # avoid_obstacle_cibernots
```
-----------------------------------------------------------------------



## license 
<a rel="license" href="https://www.apache.org/licenses/LICENSE-2.0"><img alt="Apache License" style="border-width:0" src="https://www.apache.org/img/asf-estd-1999-logo.jpg" /></a><br/>(Cibernots) </a><br/>This work is licensed under a <a rel="license" href="https://www.apache.org/licenses/LICENSE-2.0">Apache license 2.0
