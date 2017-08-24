# SENSO - SCARA PROJECT

## Part 1 - scara_v2_with_gripper - Rviz kinematic simulation
1. roslaunch scara_v2_with_gripper xacro_display_scara_rt.launch
2. roslaunch scara_v2_with_gripper xacro_display_scara.launch
3. roslaunch scara_v2_with_gripper xacro_display_rotary_table.launch

## Part 2 - scara_and_rt_moveit_config - Moveit! configuration of SCARA
1. roslaunch scara_and_rt_moveit_config demo_rviz.launch
> Run this launchfile for only kinematic simulation (in your local PC)
2. roslaunch scara_and_rt_moveit_config demo_gazebo.launch
> Run this launchfile for kinematic and dynamic simulation (in your local PC)
3. roslaunch scara_and_rt_moveit_config demo_matlab_simulink.launch
> Run this launchfile for simulation via Simulink (not really helpfull)
4. roslaunch scara_and_rt_moveit_config demo_matlab_mfile.launch
> Run this launchfile for simulation via Matlab (PC-Matlab-SLRT-SCARA)  **Most important launchfile!**

## Part 3 - gazebo_scara_and_rt - Gazebo dynamic simulation
1. **scara_v2_full_description** : Rviz kinematic simulation
    - This is used to test the SCARA manipulator (such as part 1), but here are saved the URDF models for Gazebo.
        1. roslaunch scara_v2_full_description xacro_display_scara_rt.launch
        2. roslaunch scara_v2_full_description xacro_display_scara.launch
        3. roslaunch scara_v2_full_description xacro_display_rotary_table.launch
2. **scara_v2_full_gazebo** : Gazebo world for SCARA
    - Default world (working space) for the SCARA and RT. To run default world use one of the following commands:
        1. roslaunch scara_v2_full_gazebo scara_v2_full_world.launch
        2. roslaunch scara_v2_full_gazebo scara_and_rt_world.launch.launch
        3. roslaunch scara_v2_full_gazebo rt_v2_world.launch
3. **scara_v2_full_control** : Gazebo controllers
    - In this directory the controllers for SCARA and RT are defined. To launch it use the following commands:
        1. roslaunch scara_v2_full_control scara_v2_full_control.launch
        2. roslaunch scara_v2_full_control scara_and_rt_control.launch
        3. roslaunch scara_v2_full_control rt_v2_full_control.launch

## Part 4 - scara_v2_moveit_api: Source codes for control of SCARA
1. multiple_cube_publisher.cpp
    - Publishing cubes into RViz (and Moveit simulation)
2. gripper_control.cpp
    - Gripper control in Moveit simulation
3. IK_test.cpp
    - Testing own IK solution for SCARA
4. linear_interpolation.cpp
    - Testing linear interpolation - **Doesnt work now!**
5. jointState_publisher.cpp
    - Subscribes the topic from Simulink and remakes it to sensor_msgs::JointState and publishes it -> Part 2 section 4
6. move_group_interface5_rt.cpp
    - Rotary table control node
7. move_group_interface3_PICKandPLACE.cpp
    - DEMO application with joint or position controll, but only in synchronized mode -> *No forcefeedback*
8. move_group_interface6_async_PICKandPLACE.cpp
    - DEMO application with joint or position controll in async mode -> *with forcefeddback*
