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

#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include "melfa_driver/strdef.h"

#define JOINT_NUM (8)


class MelfaHW:public hardware_interface::RobotHW
{
public:
  MelfaHW (double period);
  void update (void);
  void write (void);
  void read (void);
  void write_first (void);
  void diagnose(diagnostic_updater::DiagnosticStatusWrapper &stat);

  inline ros::Duration getPeriod()
  {
    return ros::Duration(period_);
  }
private:
  double period_;
  ros::Time time_now_;
  ros::Time time_old_;

  std::string robot_ip_;
  int socket_;
  struct sockaddr_in addr_;

  MXTCMD send_buff_;
  MXTCMD recv_buff_;
  int counter_;

  // true if use joint7
  bool use_joint7_;
  // true if use joint8
  bool use_joint8_;
  // true if joint7 is linear joint
  bool joint7_is_linear_;
  // true if joint8 is linear joint
  bool joint8_is_linear_;

  hardware_interface::JointStateInterface joint_state_interface;
  hardware_interface::PositionJointInterface joint_pos_interface;

  double cmd[JOINT_NUM];
  double pos[JOINT_NUM];
  double vel[JOINT_NUM];
  double eff[JOINT_NUM];
};
