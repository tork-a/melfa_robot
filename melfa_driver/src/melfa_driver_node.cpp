/**
 * @file melfa_driver_node.cpp
 * 
 * Driver node for MELFA robot controller
 */
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include "melfa_driver/melfa_hardware_interface.h"

/**
 * @brief Main function
 */
int main (int argc, char **argv)
{
  // init ROS node
  ros::init (argc, argv, "melfa_driver");
  ros::NodeHandle nh;

  // create hardware interface 
  MelfaHW robot;
  controller_manager::ControllerManager cm (&robot, nh);

  // set spin rate
  ros::Rate rate (1.0 / ros::Duration (0.010).toSec ());
  ros::AsyncSpinner spinner (1);
  spinner.start ();

  robot.write_first ();
  while (ros::ok ())
  {
    robot.read ();
    cm.update (ros::Time::now (), ros::Duration (0.010));
    robot.write ();
    rate.sleep ();
  }
  spinner.stop ();

  return 0;
}
