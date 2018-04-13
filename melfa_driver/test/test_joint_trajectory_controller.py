#! /usr/bin/env python
# -*- coding: utf-8 -*-
import math
import sys
import actionlib
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryActionGoal)
import rospy
import rostest
import rosnode
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
import time
import unittest


class TestJointTrajectoryController(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.init_node('test_joint_trajectory_controller_node')

    def setUp(self):
        self.joint_states_list = []

        rospy.Subscriber('joint_states',
                         JointState, self.cb_joint_states, queue_size=10)
        
        self.pub = rospy.Publisher(
            'joint_trajectory_controller/command',
            JointTrajectory,
            queue_size=10)
        
        self.client = actionlib.SimpleActionClient(
            'joint_trajectory_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction)
        self.client.wait_for_server()
        
    def cb_joint_states(self, msg):
        self.joint_states_list.append(msg)
        
    def test_joint_states(self):
        # try for 30 s
        for i in range(30):
            if not self.joint_states_list:
                time.sleep(1.0)
        self.assertTrue(self.joint_states_list)
        
if __name__ == '__main__':
    rostest.rosrun('melfa_driver',
                   'test_joint_trajectory_controller',
                   TestJointTrajectoryController)
