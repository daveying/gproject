# Introduction

Repo for my graduation project.

My master degree graduation project is about coordination control for mobile manipulator. 
My goal is to acheive movement control of manipulator and mobile base simultaneously.

# Environment

- Ubuntu 14.04
- ROS Indigo

# Package dependences

- [ur\_modern\_driver](https://github.com/ThomasTimm/ur_modern_driver) : [src/ur\_modern\_driver](https://github.com/daveying/gproject/tree/master/src/ur_modern_driver)

# Package description

- aimm_description

Description files for mobile manipulator. Source files are here: [src/aimm_description]()
  
Test this package by `cd` to `src/aimm_description/` and using command:
  
```bash
sh display.sh
```
to check the effect. You will see this if everything is alright.

![aimm description effect](https://raw.githubusercontent.com/daveying/gproject/master/doc/pic/aimm_description_effect.png)

- aimm\_moveit\_config

  MoveIt! config package for mobile manipulator
  

# How to control

## Create urdf file for mobile manipulator

TODO

## Create [MoveIt!](http://moveit.ros.org/) config package for mobile manipulator

TODO


