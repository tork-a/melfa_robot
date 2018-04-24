/**
 * @file melfa_driver_node.cpp
 * 
 * Driver node for MELFA robot controller
 */
#include <limits.h>
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include "melfa_driver/melfa_hardware_interface.h"
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

// Period [sec]
double g_period;
ros::Time g_time_now, g_time_old;
  
void loop_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  double time_dif = (g_time_now - g_time_old).toSec();
  if (time_dif > g_period * 1.2)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Periodic time exceeds 120%");
  }
  else
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Periodic time is normal");
  }
  stat.add ("Period", time_dif);
}

/**
 * @brief Main function
 */
int main (int argc, char **argv)
{
  // init ROS node
  ros::init (argc, argv, "melfa_driver");
  ros::NodeHandle nh;
  
  diagnostic_updater::Updater updater;
  updater.setHardwareID("melfa_driver");
  updater.add("Loop updater", loop_diagnostic);
  
  // Parameters
  bool realtime;
  ros::param::param<bool>("~realtime", realtime, false);
  ros::param::param<double>("~period", g_period, 0.05);

  // Setup realtime scheduler
  if (realtime)
  {
    struct sched_param param;
    memset(&param, 0, sizeof(param));
    int policy = SCHED_FIFO;
    param.sched_priority = sched_get_priority_max(policy);

    ROS_WARN("Setting up Realtime scheduler");
    if (sched_setscheduler(0, policy, &param) < 0) {
      ROS_ERROR("sched_setscheduler: %s", strerror(errno));
      ROS_ERROR("Please check you are using PREEMPT_RT kernel and set /etc/security/limits.conf");
      exit (1);
    }
    if (mlockall(MCL_CURRENT|MCL_FUTURE) < 0)
    {
      ROS_ERROR("mlockall: %s", strerror(errno));
      exit (1);
    }  
  }
  // create hardware interface 
  MelfaHW robot;
  controller_manager::ControllerManager cm (&robot, nh);

  // set spin rate
  ros::Rate rate (1.0 / ros::Duration (g_period).toSec ());
  ros::AsyncSpinner spinner (1);
  spinner.start ();

  robot.write_first ();

  while (ros::ok ())
  {
    g_time_old = g_time_now;
    g_time_now = ros::Time::now();

    // Update diagnostics
    updater.update();

    robot.read ();
    cm.update (ros::Time::now (), ros::Duration (g_period));
    robot.write ();
    rate.sleep ();
  }
  spinner.stop ();
  return 0;
}
