#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include "melfa_driver/strdef.h"

class MelfaHW:public hardware_interface::RobotHW
{
public:
  MelfaHW (void);
  void update (void);
  void write (void);
  void read (void);
  void write_first (void);

private:
  int socket_;
  struct sockaddr_in addr_;

  MXTCMD send_buff_;
  MXTCMD recv_buff_;
  int counter_;

  hardware_interface::JointStateInterface joint_state_interface;
  hardware_interface::PositionJointInterface joint_pos_interface;

  double cmd[6];
  double pos[6];
  double vel[6];
  double eff[6];
};
