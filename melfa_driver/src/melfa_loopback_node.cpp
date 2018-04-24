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
#include "melfa_driver/strdef.h"

class LoopbackNode
{
private:
  int socket_;
  struct sockaddr_in addr_;

  MXTCMD send_buff_;
  MXTCMD recv_buff_;
  
public:
  LoopbackNode()
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
  bool recv(void)
  {
    fd_set fds;
    timeval time;

    FD_ZERO (&fds);
    FD_SET (socket_, &fds);
    time.tv_sec = 1;
    time.tv_usec = 0;
    
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
      ROS_INFO("mxt.dat.jnt: % 4.3f % 4.3f % 4.3f % 4.3f % 4.3f % 4.3f",
               mxt.dat.jnt.j1,
               mxt.dat.jnt.j2,
               mxt.dat.jnt.j3,
               mxt.dat.jnt.j4,
               mxt.dat.jnt.j5,
               mxt.dat.jnt.j6);
    }
    else
    {
      ROS_ERROR("Invalid mxt.RecvType");
    }
  }
};

/**
 * @brief Main function
 */
int main (int argc, char **argv)
{
  // init ROS node
  ros::init (argc, argv, "melfa_loopback");
  ros::NodeHandle nh;
  LoopbackNode node;
  
  // Parameters
  bool realtime;
  ros::param::param<bool>("~realtime", realtime, false);
  
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

  while (ros::ok ())
  {
    node.recv();
  }
  return 0;
}
