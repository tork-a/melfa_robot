// Copyright 2018 Tokyo Opensource Robotics Kyokai Association (TORK)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file melfa_loopback_node.cpp
 * 
 * Loopback node to test melfa_driver_node
 */
#include <limits.h>
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include "melfa_driver/strdef.h"

ros::Time g_time_now, g_time_old;

class LoopbackNode
{
private:
  /// Packet counter
  int counter_;
  
  /// Periodic time [s]
  double period_;

  int socket_;
  struct sockaddr_in addr_;

  MXTCMD send_buff_;
  MXTCMD recv_buff_;

  ros::Time time_now_;
  ros::Time time_old_;
  
public:
  LoopbackNode(double period)
    : counter_(0), period_(period)
  {
    socket_ = socket(AF_INET, SOCK_DGRAM, 0);
    addr_.sin_family = AF_INET;
    addr_.sin_port = htons(10000);
    addr_.sin_addr.s_addr = inet_addr ("127.0.0.1");

    bind(socket_, (struct sockaddr *)&addr_, sizeof(addr_));
  }
  ~LoopbackNode()
  {
    close(socket_);
  }
  /**
   * @param timeout Timeout for reciving a packet [s]
   */
  bool update(double timeout=0.06)
  {
    fd_set fds;
    timeval time;

    // Record called time
    time_old_ = time_now_;
    time_now_ = ros::Time::now();

    FD_ZERO (&fds);
    FD_SET (socket_, &fds);

    time.tv_sec = 0;
    time.tv_usec = timeout * 1000;
    
    int r = select (socket_ + 1, &fds, NULL, NULL, &time);
    if (r < 0)
    {
      ROS_ERROR ("select error: %s", strerror(errno));
      ros::shutdown();
    }
    if (r > 0 && FD_ISSET (socket_, &fds))
    {
      struct sockaddr_in recv_addr;
      socklen_t len;
      
      int size = recvfrom (socket_,
                           &recv_buff_, sizeof (recv_buff_),
                           0, (struct sockaddr *)&recv_addr, &len);
      ROS_INFO("recv from: %s, size=%d", inet_ntoa(recv_addr.sin_addr), size);
      counter_ ++;
      // print recv packet
      print_mxt_packet(recv_buff_);

      // loopback
      send_buff_.Command = MXT_CMD_MOVE;
      send_buff_.SendType = MXT_TYP_JOINT;
      send_buff_.RecvType = MXT_TYP_JOINT;
      send_buff_.RecvType1 = MXT_TYP_FJOINT;
      send_buff_.RecvType2 = MXT_TYP_FB_JOINT;
      send_buff_.RecvType3 = MXT_TYP_FBKCUR;

      send_buff_.dat.jnt.j1 = recv_buff_.dat.jnt.j1;
      send_buff_.dat.jnt.j2 = recv_buff_.dat.jnt.j2;
      send_buff_.dat.jnt.j3 = recv_buff_.dat.jnt.j3;
      send_buff_.dat.jnt.j4 = recv_buff_.dat.jnt.j4;
      send_buff_.dat.jnt.j5 = recv_buff_.dat.jnt.j5;
      send_buff_.dat.jnt.j6 = recv_buff_.dat.jnt.j6;
      send_buff_.dat.jnt.j7 = recv_buff_.dat.jnt.j7;
      send_buff_.dat.jnt.j8 = recv_buff_.dat.jnt.j8;

      size = sendto (socket_,
                     (char *) &send_buff_, sizeof (send_buff_),
                     0, (struct sockaddr *) &recv_addr, sizeof (recv_addr));
      if (size != sizeof (send_buff_))
      {
        ROS_WARN ("Failed to send packet.");
      }
    }
  }
  void print_mxt_packet(MXTCMD& mxt)
  {
    ROS_INFO("mxt.Command: %d", mxt.Command);
    ROS_INFO("mxt.SendType: %d", mxt.SendType);
    ROS_INFO("mxt.RecvType: %d", mxt.RecvType);
    ROS_INFO("mxt.RecvType1: %d", mxt.RecvType1);
    ROS_INFO("mxt.RecvType2: %d", mxt.RecvType2);
    ROS_INFO("mxt.RecvType3: %d", mxt.RecvType3);
    if (mxt.RecvType == MXT_TYP_JOINT)
    {
      ROS_INFO("mxt.dat.jnt: % 4.3f % 4.3f % 4.3f % 4.3f % 4.3f % 4.3f % 4.3f % 4.3f",
               mxt.dat.jnt.j1,
               mxt.dat.jnt.j2,
               mxt.dat.jnt.j3,
               mxt.dat.jnt.j4,
               mxt.dat.jnt.j5,
               mxt.dat.jnt.j6,
               mxt.dat.jnt.j7,
               mxt.dat.jnt.j8);
    }
    else
    {
      ROS_ERROR("Invalid mxt.RecvType");
    }
  }
  void diagnose(diagnostic_updater::DiagnosticStatusWrapper &stat)
  {
    stat.add ("Counter", counter_);
    
    double diff = (time_now_ - time_old_).toSec();
    // Check over-run
    if (diff > period_ * 1.2)
    {
      stat.summary(diagnostic_msgs::DiagnosticStatus::WARN,
                   "Periodic time exceeds 120%");
    }
    else
    {
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK,
                   "Periodic time is normal");
    }
    stat.add ("Period", diff);
  }
};

void loop_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  double time_dif = (g_time_now - g_time_old).toSec();
  stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Periodic time is normal");
  stat.add ("Period", time_dif);
}

/**
 * @brief Main function
 */
int main (int argc, char **argv)
{
  // init ROS node
  ros::init (argc, argv, "melfa_loopback");
  ros::NodeHandle nh;
  // Parameters
  bool realtime;
  ros::param::param<bool>("~realtime", realtime, false);
  double period;
  ros::param::param<double>("~period", period, 0.0071);

  // Loopback node
  LoopbackNode node(period);

  // Diagnostics
  diagnostic_updater::Updater updater;
  updater.setHardwareID("melfa_loopback");
  updater.add("diagnose", &node, &LoopbackNode::diagnose);
  
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
  // Set spin rate frequency
  ros::Rate rate (1.0 / period);
  while (ros::ok ())
  {
    // Recieve and response
    node.update();
    // Update diagnostics
    updater.update();
    // Sleep for next cycle
    rate.sleep();
  }
  return 0;
}
