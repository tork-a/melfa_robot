^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package melfa_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.2 (2018-12-16)
------------------
* Add LICENSE and notification on source code(`#18 <https://github.com/tork-a/melfa_robot/issues/18>`_)
* Contributors: Ryosuke Tajima

0.0.4 (2019-02-26)
------------------
* Fix wrong install location(`#25 <https://github.com/tork-a/melfa_robot/issues/25>`_)
 - Reserve test permissions
* Fix minor typo(`#20 <https://github.com/tork-a/melfa_robot/issues/20>`_)
* Contributors: Ryosuke Tajima, Wolf Vollprecht

0.0.1 (2018-12-14)
------------------
* Remove check.sh for eliminating catkin_lint warning
* Add jointx_is_linear parameter for the linear base(`#17 <https://github.com/tork-a/melfa_robot/issues/17>`_)
* rename melfa_ros to melfa_robot
* Add additional joints(`#13 <https://github.com/tork-a/melfa_robot/issues/13>`_)
* Add use_joint7 and use_joint8 parameters
 - Remove dummy joints from rv7fl.urdf.xacro
* Pull request for the first delivery(`#5 <https://github.com/tork-a/melfa_robot/issues/5>`_)
* Change author email
* Expand joint number to 8 only for acutal robot check
* Add realtime feature and docs(`#3 <https://github.com/tork-a/melfa_robot/issues/3>`_)
* Improve driver and loopback node, add diagnostics
  - Make loopback node periodic considering the hardware specs
  - Add diagnotics to see period over-run
* Disable realtime in tests
* Make loopback node realtime
* Add realtime feature and docs
* Change test to wait topic for 30s
* Add more dependencies
* Check node list before test
* Add package dependencies
* Add .travis.yml for CI
* Add moveit configuration files(`#2 <https://github.com/tork-a/melfa_robot/issues/2>`_)
  - Confirmed to work with RT-ToolBox
* Add URDF and mesh files(`#1 <https://github.com/tork-a/melfa_robot/issues/1>`_)
* Change driver to use melfa_description
* Add ROS_INFO for robot_ip
* Improve UDP error handling not to abort
* Add first test
* Fix integer size incompatibility of the packet
* Add melfa_loopback\node for test
* First commit
  - Proof of function
  - This can control RT-Toolbox3 simulator
  - All parameters are hard-coded
* Contributors: Ryosuke Tajima
