#include "melfa_driver/melfa_hardware_interface.h"

MelfaHW::MelfaHW ()
{
  // initialize UDP socket
  for (;;)
  {
    socket_ = socket (AF_INET, SOCK_DGRAM, 0);
    if (socket_ > 0)
    {
      break;
    }
    ROS_ERROR ("Waiting for opening socket");
    sleep(1);
  }
  ROS_INFO ("Socket opened");

  // set IP and port
  ros::param::param<std::string>("~robot_ip", robot_ip_, "127.0.0.1");

  addr_.sin_family = AF_INET;
  addr_.sin_port = htons (10000);
  addr_.sin_addr.s_addr = inet_addr (robot_ip_.c_str());

  counter_ = 0;
  memset (&send_buff_, 0, sizeof (send_buff_));
  memset (&recv_buff_, 0, sizeof (recv_buff_));

  std::string joint_names[6] =
  {
  "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
  // connect and register the joint state interface
  for (int i = 0; i < 6; i++)
  {
    hardware_interface::JointStateHandle state_handle (joint_names[i], &pos[i], &vel[i], &eff[i]);
    joint_state_interface.registerHandle (state_handle);
  }
  registerInterface (&joint_state_interface);

  // connect and register the joint position interface
  for (int i = 0; i < 6; i++)
  {
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

  memset (&recv_buff_, 0, sizeof (recv_buff_));

  FD_ZERO (&fds);
  FD_SET (socket_, &fds);

  time.tv_sec = 1;
  time.tv_usec = 0;

  int status = select (socket_ + 1, &fds, (fd_set *) NULL, (fd_set *) NULL, &time);
  if (status < 0)
  {
    ROS_ERROR ("Cannot recieve packet");
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
    if (counter_ == 0)
    {
      for (int i = 0; i < 6; i++)
      {
        cmd[i] = pos[i];
      }
    }
    ROS_INFO ("%f %f %f %f %f %f", joint->j1, joint->j2, joint->j3, joint->j4, joint->j5, joint->j6);
    counter_++;
  }
  else
  {
    ROS_ERROR ("recvfrom failed");
  }
}

void MelfaHW::update (void)
{
  ROS_INFO ("update");
}
