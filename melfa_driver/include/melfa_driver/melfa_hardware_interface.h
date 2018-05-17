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

  hardware_interface::JointStateInterface joint_state_interface;
  hardware_interface::PositionJointInterface joint_pos_interface;

  double cmd[JOINT_NUM];
  double pos[JOINT_NUM];
  double vel[JOINT_NUM];
  double eff[JOINT_NUM];
};
