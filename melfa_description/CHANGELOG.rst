^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package melfa_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.2 (2018-12-16)
------------------
* Add LICENSE and notification on source code(`#18 <https://github.com/tork-a/melfa_robot/issues/18>`_ )
* Contributors: Ryosuke Tajima

0.0.4 (2019-02-26)
------------------

0.0.1 (2018-12-14)
------------------
* Remove catkin_lint error from melfa_description
* Add copyright holder of mesh files in README.md
* Reorganize URDF for RV4FL to be re-usable
* Add additional joints(`#13 <https://github.com/tork-a/melfa_robot/issues/13>`_)
* Organize rv7fl.urdf.xacro using xacro
* Add use_joint7 and use_joint8 parameters
  - Remove dummy joints from rv7fl.urdf.xacro
* Add RV4FL mesh data(`#11 <https://github.com/tork-a/melfa_robot/issues/11>`_)
* Add missing dependencies
* Add mesh license and README.md
* Modify the package meta files
* Add mesh files for RV7FL
  - They are modified its origin, rotation in mesh
  - Scale are modified in URDF
* Modify URDF to adopt the package structure
  - Add robot argument to specify target robot
* Add RV4FL mesh data
* Pull request for the first delivery(`#5 <https://github.com/tork-a/melfa_robot/issues/5>`_)
* Remove mesh related files from melfa_description
* Add dummy joint7 and joint8 for actual robot
* Add use_gui argument and default is false
* Add URDF and mesh files (`#1 <https://github.com/tork-a/melfa_robot/issues/1>`_)
* Modify joint limits to fit the spec sheet
* Fix mesh orientation, colors
* Change driver to use melfa_description
* Fix mesh size unit, urdf
* Contributors: KazukiHiraizumi, Ryosuke Tajima
