# Introduction

Repo for my graduation project.

My master degree graduation project is about coordination control for mobile manipulator. 
My goal is to acheive movement control of manipulator and mobile base simultaneously.

# Environment

- Ubuntu 14.04
- ROS Indigo

# Package dependences

- [ur\_modern\_driver](https://github.com/ThomasTimm/ur_modern_driver),
Source at src/ur\_modern\_driver

- [hokuyo_node](http://wiki.ros.org/hokuyo_node),
You can install hokuyo_node via command line:
```bash
sudo apt-get install ros-indigo-hokuyo-node
```
- [laser_scan_matcher](http://wiki.ros.org/laser_scan_matcher),
This is a node from [scan_tools](http://wiki.ros.org/scan_tools?distro=indigo), source at src/scan\_tools/laser\_scan_matcher

# Package description

## ur_modern_driver

Provide the velocity control interface for ur.

## aimm_description

Description files for mobile manipulator. Source files are here: [src/aimm_description]()
  
Test this package by `cd` to `src/aimm_description/` and using command:
  
```bash
sh display.sh
```
You will see the effect like below if everything is alright.

![aimm description effect](https://raw.githubusercontent.com/daveying/gproject/master/doc/pic/aimm_description_effect.png)

## aimm\_moveit\_config

MoveIt! config package for mobile manipulator
  
## laser_scan_matcher

Launch this module with a custom launch file:
```bash
roslaunch laser_scan_matcher localization.launch
```
## ur5_jacobian



# How to control

## Create urdf file for mobile manipulator

TODO

## Create [MoveIt!](http://moveit.ros.org/) config package for mobile manipulator

TODO


