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

#include "melfa_driver/melfa_hardware_interface.h"

MelfaHW::MelfaHW (double period)
  : counter_(0), period_(period),
    use_joint7_(false), use_joint8_(false),
    joint7_is_linear_(false), joint8_is_linear_(false)
{
  // initialize UDP socket
  socket_ = socket (AF_INET, SOCK_DGRAM, 0);
  if (socket_ < 0)
  {
    ROS_ERROR ("Cannot open socket");
    exit(1);
  }
  // set IP and port
  ros::param::param<std::string>("~robot_ip", robot_ip_, "127.0.0.1");
  // Usage of additional joints
  ros::param::param<bool>("~use_joint7", use_joint7_, false);
  ros::param::param<bool>("~use_joint8", use_joint8_, false);
  ros::param::param<bool>("~joint7_is_linear", joint7_is_linear_, false);
  ros::param::param<bool>("~joint8_is_linear", joint8_is_linear_, false);

  addr_.sin_family = AF_INET;
  addr_.sin_port = htons (10000);
  addr_.sin_addr.s_addr = inet_addr (robot_ip_.c_str());

  memset (&send_buff_, 0, sizeof (send_buff_));
  memset (&recv_buff_, 0, sizeof (recv_buff_));

  std::string joint_names[JOINT_NUM] =
    {
      "joint1", "joint2", "joint3", "joint4",
      "joint5", "joint6", "joint7", "joint8"
    };
  // connect and register the joint state interface
  for (int i = 0; i < JOINT_NUM; i++)
  {
    // Skip unused joints
    if (i==6 and not use_joint7_)
    {
      continue;
    }
    if (i==7 and not use_joint8_)
    {
      continue;
    }
    hardware_interface::JointStateHandle state_handle (joint_names[i], &pos[i], &vel[i], &eff[i]);
    joint_state_interface.registerHandle (state_handle);
  }
  registerInterface (&joint_state_interface);

  // connect and register the joint position interface
  for (int i = 0; i < JOINT_NUM; i++)
  {
    // Skip unused joints
    if (i==6 and not use_joint7_)
    {
      continue;
    }
    if (i==7 and not use_joint8_)
    {
      continue;
    }
    hardware_interface::JointHandle joint_pos_handle (joint_state_interface.getHandle (joint_names[i]), &cmd[i]);
    joint_pos_interface.registerHandle (joint_pos_handle);
  }
  registerInterface (&joint_pos_interface);
}

void MelfaHW::write_first (void)
{
  memset (&send_buff_, 0, sizeof (send_buff_));
  // Get current joint angles in 
  send_buff_.Command = MXT_CMD_NULL;
  send_buff_.SendType = MXT_TYP_NULL;
  send_buff_.RecvType = MXT_TYP_JOINT;
  send_buff_.SendIOType = MXT_IO_NULL;
  send_buff_.RecvIOType = MXT_IO_NULL;
  send_buff_.BitTop = 0;
  send_buff_.BitMask = 0;
  send_buff_.IoData = 0;
  send_buff_.CCount = 0;

  int size = sendto (socket_, (char *) &send_buff_, sizeof (send_buff_), 0, (struct sockaddr *) &addr_, sizeof (addr_));
  if (size != sizeof (send_buff_))
  {
    ROS_ERROR ("Connot send packet to MELFA controller. Check the configuration.");
    exit (1);
  }
}

void MelfaHW::write (void)
{
  // Send MOVE command
  memset (&send_buff_, 0, sizeof (send_buff_));
  send_buff_.Command = MXT_CMD_MOVE;
  send_buff_.SendType = MXT_TYP_JOINT;
  send_buff_.RecvType = MXT_TYP_JOINT;
  send_buff_.RecvType1 = MXT_TYP_FJOINT;
  send_buff_.RecvType2 = MXT_TYP_FB_JOINT;
  send_buff_.RecvType3 = MXT_TYP_FBKCUR;

  send_buff_.SendIOType = MXT_IO_NULL;
  send_buff_.RecvIOType = MXT_IO_NULL;
  send_buff_.BitTop = 0;
  send_buff_.BitMask = 0;
  send_buff_.IoData = 0;
  send_buff_.dat.jnt.j1 = cmd[0];
  send_buff_.dat.jnt.j2 = cmd[1];
  send_buff_.dat.jnt.j3 = cmd[2];
  send_buff_.dat.jnt.j4 = cmd[3];
  send_buff_.dat.jnt.j5 = cmd[4];
  send_buff_.dat.jnt.j6 = cmd[5];
  if (joint7_is_linear_)
  {
    // Convert unit to mm
    send_buff_.dat.jnt.j7 = cmd[6] * 1000.0;
  }
  else
  {
    send_buff_.dat.jnt.j7 = cmd[6];
  }
  if (joint8_is_linear_)
  {
    // Convert unit to [mm]
    send_buff_.dat.jnt.j8 = cmd[7] * 1000.0;
  }
  else
  {
    send_buff_.dat.jnt.j8 = cmd[7];
  }
  send_buff_.CCount = counter_;

  int size = sendto (socket_, (char *) &send_buff_, sizeof (send_buff_), 0, (struct sockaddr *) &addr_, sizeof (addr_));
  if (size != sizeof (send_buff_))
  {
    ROS_ERROR ("Connot send packet to MELFA controller. Check the configuration.");
    exit (1);
  }
}

void MelfaHW::read (void)
{
  fd_set fds;
  timeval time;

  // Record time
  time_old_ = time_now_;
  time_now_ = ros::Time::now();

  memset (&recv_buff_, 0, sizeof (recv_buff_));

  FD_ZERO (&fds);
  FD_SET (socket_, &fds);

  time.tv_sec = 0;
  time.tv_usec = 2 * period_ * 1000000;

  int status = select (socket_+1, &fds, (fd_set *) NULL, (fd_set *) NULL, &time);
  if (status < 0)
  {
    ROS_ERROR ("Cannot receive packet");
  }
  if ((status > 0) && FD_ISSET (socket_, &fds))
  {
    int size = recvfrom (socket_, &recv_buff_, sizeof (recv_buff_), 0, NULL, NULL);
    if (size < 0)
    {
      ROS_ERROR ("recvfrom failed");
      exit (1);
    }
    JOINT *joint = (JOINT *) & recv_buff_.dat;
    pos[0] = joint->j1;
    pos[1] = joint->j2;
    pos[2] = joint->j3;
    pos[3] = joint->j4;
    pos[4] = joint->j5;
    pos[5] = joint->j6;

    if (joint7_is_linear_)
    {
      // Convert unit to [m]
      pos[6] = joint->j7 / 1000.0;
    }
    else
    {
      pos[6] = joint->j7;
    }
    
    if (joint8_is_linear_)
    {
      // Convert unit to [m]
      pos[7] = joint->j8 / 1000.0;
    }
    else
    {
      pos[7] = joint->j8;
    }
    // Set current position when it starts
    if (counter_ == 0)
    {
      for (int i = 0; i < JOINT_NUM; i++)
      {
        cmd[i] = pos[i];
      }
    }
    counter_++;
  }
  else
  {
    ROS_WARN ("Failed to receive packet.");
  }
}

void MelfaHW::diagnose(diagnostic_updater::DiagnosticStatusWrapper &stat)
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
