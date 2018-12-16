# melfa_robot [![Build Status](https://travis-ci.com/tork-a/melfa_robot.svg?token=Eg7EHKJ8kwE5VZs6TwDp&branch=master)](https://travis-ci.com/tork-a/melfa_robot)

This package is to control the MELFA's robot arms using ROS.

## Supported hardware

### Robot controllers

Currently `melfa_driver` is checked with the following MELFA robot
controller.

<img src="http://www.mitsubishielectric.co.jp/fa/products/rbt/robot/pmerit/common/img/src/s_cr750_751.jpg" width="200x"> CR750-Q 

### Robot arms

Currently `melfa_description` contains the models of following MELFA robots.

<img src="http://www.mitsubishielectric.co.jp/fa/products/rbt/robot/pmerit/common/img/src/s_4f.jpg" width="200x"> RV4FL

<img src="http://www.mitsubishielectric.co.jp/fa/products/rbt/robot/pmerit/common/img/src/s_7f.jpg" width="200x"> RV7FL

## Quick start with the loopback node

`melfa_driver/melfa_driver_node` is the controller node, providing
`hardware_interface::RobotHW`. This controller communicate with the
actual robot controller, or the simulater named "RT Toolbox3" on
Windowns via Ethernet(TCP/IP).

In case that you don't have actual robot and simulator, the package
contains the `loopback(dummy) node` mimicing the robot controller. It is 
`melfa_driver/melfa_loopback_node`. This node set the current joint angle identical to commanded value, and return them as current state.

When you want to use the loopback node, launch the system as:

```
$ roslaunch melfa_driver melfa_driver.launch loopback:=true
```

If you want view the robot model, launch rviz as:

```
$ roslaunch melfa_description rviz.launch use_gui:=false
```

You can use rqt plug-in `JointTrajctoryController` to control the
robot joints using slider GUI.

```
$ rqt -s rqt_joint_trajectory_controller/JointTrajectoryController
```

## MoveIt!

You can try MoveIt! Rviz plug-in.

```
$ roslaunch rv7fl_moveit_config moveit_planning_execution.launch 
```

Currently there are moveit_config packages for the following robot.

<img src="img/moveit_rv4fl.png" width="200x"> RV4FL

<img src="img/moveit_rv7fl.png" width="200x"> RV7FL

# License

- [YOODS Inc.](https://www.yoods.co.jp/) holds the copyright of all
  mesh files in ``melfa_description/mesh`` directory which are
  licensed under a Creative Commons Attribution-ShareAlike 4.0
  International License.
- Other parts are licenced under Apache License 2.0.
