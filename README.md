# melfa_ros [![Build Status](https://travis-ci.com/tork-a/melfa_ros.svg?token=Eg7EHKJ8kwE5VZs6TwDp&branch=master)](https://travis-ci.com/tork-a/melfa_ros)

This package is for control the MELFA's robot arms from ROS.

## Quick start with the loopback node

`melfa_driver/melfa_driver_node` is the controller node, providing
`hardware_interface::RobotHW`. This controller communicate with the
actual robot controller or the simulater named "RT Toolbox3" on
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
$ roslaunch melfa_description rviz.launch 
```

You can use rqt plug-in `JointTrajctoryController` to control the
robot joints using slider GUI.

```
$ rqt -s rqt_joint_trajectory_controller/JointTrajectoryController
```

You can try MoveIt! rviz plug-in.

```
$ roslaunch rv7fl_moveit_config moveit_planning_execution.launch 
```

