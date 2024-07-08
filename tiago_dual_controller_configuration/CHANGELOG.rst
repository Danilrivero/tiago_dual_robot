^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tiago_dual_controller_configuration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* change module name into 20\_*
* change the bases config file for their packages config files
* Contributors: Aina

2.0.13 (2024-06-26)
-------------------
* Merge branch 'dtk/move-robot-args' into 'humble-devel'
  Change import for launch args
  See merge request robots/tiago_dual_robot!111
* Change import for launch args
* Contributors: David ter Kuile, davidterkuile

2.0.12 (2024-06-25)
-------------------
* Merge branch 'dtk/fix/urdf' into 'humble-devel'
  Dtk/fix/urdf
  See merge request robots/tiago_dual_robot!109
* Load correct joint state params
* Add imu gazebo and imo sensor broadcaster
* Merge branch 'fix/remove_extra_joints' into 'humble-devel'
  Remove casters and suspension from joint_state_broadcaster
  See merge request robots/tiago_dual_robot!110
* Remove casters and suspension from joint_state_broadcaster
* Contributors: David ter Kuile, Jordan Palacios, Noel Jimenez, davidterkuile

2.0.11 (2024-06-10)
-------------------
* Merge branch 'fix/end_effector_controller' into 'humble-devel'
  fix end_effector_controller whne no-ft-sensor
  See merge request robots/tiago_dual_robot!105
* fix end_effector_controller whne no-ft-sensor
* Contributors: Aina, davidterkuile

2.0.10 (2024-05-16)
-------------------

2.0.9 (2024-05-09)
------------------

2.0.8 (2024-05-09)
------------------

2.0.7 (2024-05-09)
------------------

2.0.6 (2024-05-08)
------------------
* Merge branch 'omm/fix/deps_and_joy' into 'humble-devel'
  New joy config and missing deps added
  See merge request robots/tiago_dual_robot!99
* Adding base_type to modules
* Contributors: davidterkuile, oscarmartinez

2.0.5 (2024-05-07)
------------------
* Merge branch 'omm/fix/complete_standarization' into 'humble-devel'
  Final standarization
  See merge request robots/tiago_dual_robot!98
* Finished controllers std
* Name and deps issue
* Configs updated
* Launch files standarization
* Contributors: Oscar, davidterkuile, oscarmartinez

2.0.4 (2024-04-10)
------------------
* Add ros2controlcli dependency
* Contributors: Noel Jimenez

2.0.3 (2024-03-06)
------------------

2.0.2 (2024-02-28)
------------------
* add the posibility of using robotiq end effector with its controller
* deleting unnecessary config files
* Remove unused param controller type
* Contributors: Aina Irisarri, David ter Kuile

2.0.1 (2024-01-18)
------------------

2.0.0 (2024-01-18)
------------------
* Merge branch 'ros2-tiago-gazebo' into 'humble-devel'
  Ros2 tiago gazebo
  See merge request robots/tiago_dual_robot!81
* Add maintainer
* Update authors
* Remove unused param mobile_base_controller
* Updat end-effector-controller
* CMake version to 3.8
* Bump module number for omni base
* update to  new robot argument method
* Fix tests and remove unused files
* Update launch file layout and use new scoped_launch_file function
* Merge branch 'fix/module' into 'ros2-tiago-gazebo'
  Set 'finishes: True' for default_controllers module
  See merge request robots/tiago_dual_robot!82
* Set 'finishes: True' for default_controllers module
* Add modules for module manager
* Add website tag
* Add motions files(still old param style) and controller yaml
* Add general motions and removed schunk-wsg from the regen_em scripts
* Add default controllers
* Cmakelists.txt and package.xml only
* Merge branch 'change_license_to_apache' into 'erbium-devel'
  change public license to Apache License 2.0
  See merge request robots/tiago_dual_robot!77
* change public license to Apache License 2.0
* Contributors: David ter Kuile, Jordan Palacios, Noel Jimenez, Sai Kishor Kothakota, davidterkuile

0.4.9 (2023-04-18)
------------------

0.4.8 (2023-03-13)
------------------

0.4.7 (2023-01-23)
------------------

0.4.6 (2022-11-16)
------------------
* Merge branch 'fix/enable-odom-tf' into 'erbium-devel'
  fix enable_odom_tf
  See merge request robots/tiago_dual_robot!63
* Update mobile_base_controller.yaml
* Contributors: josegarcia

0.4.5 (2022-10-24)
------------------
* Merge branch 'feat/robust-odometry-integration' into 'erbium-devel'
  disabled odom tf publication
  See merge request robots/tiago_dual_robot!61
* disabled odom tf publication
* Contributors: josegarcia

0.4.4 (2022-08-10)
------------------
* Merge branch 'fix_torso_collision' into 'erbium-devel'
  Fix collision boxes for the torso
  See merge request robots/tiago_dual_robot!59
* Reorder param order for making it easier to spot
* Remove extra joints because were changed to fixed
* Contributors: saikishor, thomaspeyrucain

0.4.3 (2022-07-26)
------------------
* Merge branch 'omnibase_wbc' into 'erbium-devel'
  propagate base_type to wbc
  See merge request robots/tiago_dual_robot!58
* propagate base_type to wbc
* Contributors: Sai Kishor Kothakota, davidfernandez

0.4.2 (2022-07-21)
------------------
* Merge branch 'add_omni_tiago_dual' into 'erbium-devel'
  Add special motions for Tiago dual omni to not collide with the base
  See merge request robots/tiago_dual_robot!55
* Add base_type to the missing launch files
* Contributors: saikishor, thomaspeyrucain

0.4.1 (2022-07-14)
------------------
* Merge branch 'fix_passed_params' into 'erbium-devel'
  Added ft_sensor params
  See merge request robots/tiago_dual_robot!57
* Added ft_sensor params
* Contributors: Narcis Miguel, saikishor

0.4.0 (2022-05-03)
------------------
* Merge branch 'no-end-effector-bugfix' into 'erbium-devel'
  No end effector bugfix
  See merge request robots/tiago_dual_robot!54
* bools to true
* update for tiago missing one of the arms
* minor bugfixes in launch files
* minor bugfixes in launch files
* generate new config files
* minor bugfixes in launch files
* generate new config files
* minor bugfixes in launch files
* generate new config files
* Contributors: David ter Kuile, saikishor

0.3.8 (2022-03-23)
------------------

0.3.7 (2022-03-18)
------------------
* Merge branch 'add_robotiq_epick_gripper' into 'erbium-devel'
  Add robotiq-epick gripper to tiago dual
  See merge request robots/tiago_dual_robot!51
* Remove open/close both buttons for epick + add cartesian impedance cotroller support for epick + add effort package
* Change controller and joint name and adapt the joystick and the motions for the epick gripper
* Add robotiq-epick config files
* Contributors: davidfernandez, thomaspeyrucain

0.3.6 (2021-12-22)
------------------
* Merge branch 'reduced_wbc' into 'erbium-devel'
  Add argument to set torso, left_arm, right_arm to no control
  See merge request robots/tiago_dual_robot!50
* Add argument to set torso, left_arm, right_arm to no control
* Contributors: Adria Roig, narcismiguel

0.3.5 (2021-11-26)
------------------

0.3.4 (2021-11-22)
------------------
* Merge branch 'fix-omnibase' into 'erbium-devel'
  Added speed limits for the space velocity filter
  See merge request robots/tiago_dual_robot!48
* Added speed limits for the space velocity filter
* Merge branch 'conditional_dependencies' into 'erbium-devel'
  Conditional dependencies
  See merge request robots/tiago_dual_robot!47
* added PAL_DISTRO conditioning for PAL dependencies
* change to package version 3
* Contributors: Sai Kishor Kothakota, antoniobrandi, saikishor, victor

0.3.3 (2021-11-10)
------------------
* added cartesian_impedance_controller controller as dependency
* Contributors: saikishor

0.3.2 (2021-11-10)
------------------
* Merge branch 'omni_base_fix' into 'erbium-devel'
  reduced velocity in order to avoid a wheel blocking due to current limits
  See merge request robots/tiago_dual_robot!46
* reduced velocity in order to avoid a wheel blocking due to current limits
* Contributors: antoniobrandi, saikishor

0.3.1 (2021-11-09)
------------------
* Merge branch 'cartesian_impedance' into 'erbium-devel'
  Cartesian impedance
  See merge request robots/tiago_dual_robot!42
* added joint 5 motor torque constant and increased the gain
* added gain parameters
* minor fixes
* added the cartesian_impedance_controller to tiago_dual_controllers launch
* added files for the setup of the cartesian impedance controller
* Contributors: Sai Kishor Kothakota, saikishor

0.3.0 (2021-11-03)
------------------
* Merge branch 'omni_base_robot' into 'erbium-devel'
  Creating tiago dual with omni base robot
  See merge request robots/tiago_dual_robot!44
* updated configuration for tiago with omni base
* bringup of the tiago dual with omni base
* Contributors: antoniobrandi, saikishor

0.2.3 (2021-08-31)
------------------
* Merge branch 'kangaroo_wbc' into 'erbium-devel'
  Add BS pararameters for local joint control
  See merge request robots/tiago_dual_robot!43
* Add BS pararameters for local joint control
* Contributors: Adria Roig, victor

0.2.2 (2021-08-06)
------------------
* Merge branch 'robotiq-impedance-issues' into 'erbium-devel'
  fix: missing chain definition for robotiq gripper
  See merge request robots/tiago_dual_robot!41
* refact: unify condition
* fix: missing chain definition for robotiq gripper
* Contributors: daniellopez, saikishor

0.2.1 (2021-06-01)
------------------
* Merge branch 'impedance_controllers' into 'erbium-devel'
  Impedance controllers
  See merge request robots/tiago_dual_robot!37
* fix the model chains naming
* fix: endless loop in joint 6
* added the missing motor_torque_constant for arm_1 joints
* Fix the actuator names in the config files
* added joint_impedance_trajectory_controller dependency
* added impedance controllers launch and only loading of controllers at startup
* added impedance trajectory controllers configuration file
* Contributors: Sai Kishor Kothakota, daniellopez, victor

0.2.0 (2021-05-06)
------------------
* Merge branch 'robotiq_gripper' into 'erbium-devel'
  Robotiq gripper
  See merge request robots/tiago_dual_robot!39
* add the robotiq grippers to the tests and added dependencies
* generate gravity compensation configuration
* load the joint trajectory controller for robotiq grippers
* added joint trajectory controller configurations for robotiq 2F-85 and robotiq-140
* remove redundant regen_em_file script
* Contributors: Sai Kishor Kothakota, saikishor

0.1.37 (2021-03-29)
-------------------
* Merge branch 'cutom-end-effector' into 'erbium-devel'
  Cutom end effector
  See merge request robots/tiago_dual_robot!38
* chore: delete spaces
* chore: adapt em file for custom
* chore: add combinations for custom
* feat: create regen_em_file
* Contributors: daniellopez, davidfernandez

0.1.36 (2021-01-12)
-------------------
* Merge branch 'missing_safety_files' into 'erbium-devel'
  added missing safety files for the wrist and torso joints
  See merge request robots/tiago_dual_robot!36
* Merge branch 'gravityfix' into 'missing_safety_files'
  Add gravity mode for new wrist model on tiagodual
  See merge request robots/tiago_dual_robot!35
* fix left/right wrist
* Add gravity mode for new wrist model on tiagodual
* added missing safety files for the wrist and torso joints
* Contributors: Irina Cocolos, Sai Kishor Kothakota, victor

0.1.35 (2021-01-12)
-------------------

0.1.34 (2020-11-25)
-------------------

0.1.33 (2020-10-21)
-------------------

0.1.32 (2020-09-08)
-------------------

0.1.31 (2020-08-03)
-------------------

0.1.30 (2020-07-30)
-------------------

0.1.29 (2020-07-27)
-------------------
* Merge branch 'safety_parameters' into 'erbium-devel'
  Update default_safety_parameters.yaml with new changes in the safety of local joint control
  See merge request robots/tiago_dual_robot!31
* Update default_safety_parameters.yaml with new changes in the safety of local joint control
* Contributors: saikishor, victor

0.1.28 (2020-07-10)
-------------------

0.1.27 (2020-07-01)
-------------------

0.1.26 (2020-06-19)
-------------------

0.1.25 (2020-06-06)
-------------------
* Merge branch 'fix-dual-ft' into 'erbium-devel'
  Fix dual ft
  See merge request robots/tiago_dual_robot!27
* fix dual stuff
* fix arguments for dual related to ft left and right
* Contributors: daniellopez, victor

0.1.24 (2020-06-02)
-------------------

0.1.23 (2020-05-28)
-------------------

0.1.22 (2020-05-27)
-------------------

0.1.21 (2020-05-12)
-------------------

0.1.20 (2020-05-06)
-------------------

0.1.19 (2020-04-21)
-------------------

0.1.18 (2020-04-20)
-------------------

0.1.17 (2020-04-20)
-------------------

0.1.16 (2020-04-16)
-------------------

0.1.15 (2020-04-08)
-------------------

0.1.14 (2020-03-25)
-------------------

0.1.13 (2020-03-23)
-------------------

0.1.12 (2020-01-28)
-------------------
* Merge branch 'specifics_file' into 'erbium-devel'
  added missing actuator specifics file
  See merge request robots/tiago_dual_robot!14
* added missing actuator specifics file
* Contributors: Sai Kishor Kothakota, Victor Lopez

0.1.11 (2020-01-08)
-------------------
* Added wbc_controllers launch file
* Contributors: Jordan Palacios

0.1.10 (2019-11-06)
-------------------

0.1.9 (2019-10-03)
------------------
* Merge branch 'wbc' into 'erbium-devel'
  Add local joint control launch file for WBC
  See merge request robots/tiago_dual_robot!11
* Add local joint control launch file for WBC
* Contributors: Adria Roig, Victor Lopez

0.1.8 (2019-10-02)
------------------

0.1.7 (2019-09-27)
------------------

0.1.6 (2019-09-26)
------------------

0.1.5 (2019-09-05)
------------------

0.1.4 (2019-06-07)
------------------
* Merge branch 'torso_controller_tol' into 'erbium-devel'
  Fix bug in torso controller tolerances
  See merge request robots/tiago_dual_robot!5
* Fix bug in torso controller tolerances
* Contributors: Adria Roig, Victor Lopez

0.1.3 (2019-05-22)
------------------
* Fix gravity compensation with 2 arms
* Merge branch 'arm-update' into 'erbium-devel'
  Arm update
  See merge request robots/tiago_dual_robot!4
* Fix gravity compensation reduction ratios
* Contributors: Victor Lopez

0.1.2 (2019-05-02)
------------------

0.1.1 (2019-04-16)
------------------
* Use tiago dual moveit group
* Contributors: Victor Lopez

0.1.0 (2019-04-15)
------------------
* Fix package versions
* Merge branch 'tiago-dual' into 'master'
  Tiago dual
  See merge request robots/tiago_dual_robot!1
* Add missing tiago dependencies
* Continue creation of tiago_dual_robot
* First functional version
* Contributors: Victor Lopez
