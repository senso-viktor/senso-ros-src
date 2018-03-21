# SENSO - SCARA PROJECT
*This project was designed for German Company SENSODRIVE GmbH. You can find more 
about sensodrive on* [SENSODRIVEs website](https://www.sensodrive.de/EN/) <br />
*This project was designed by Viktor Dluhos (viktordluhos@gmail.com) a student of 
Slovak University of Technology, Faculty of Electrical Engineering and Information
Technology, [Institute of Robotics and Cybernetics](http://www.urk.fei.stuba.sk/en) 
with the help of **doc. Ing. František Duchoň, PhD. - frantisek.duchon@stuba.sk** 
and [**National Centre of Robotics**](http://nacero.sk/language/en/)*

## Part 1 - scara_v2_with_gripper - Rviz kinematic simulation
Run this launchfile for kinematic simulation of **SCARA and Rotary table**
```
roslaunch scara_v2_with_gripper xacro_display_scara_rt.launch
```
Run this launchfile for kinematic simulation of **just for SCARA**
```
roslaunch scara_v2_with_gripper xacro_display_scara.launch
```
Run this launchfile for kinematic simulation of **just for Rotary Table**
```
roslaunch scara_v2_with_gripper xacro_display_rotary_table.launch
```
## Part 2 - scara_and_rt_moveit_config - Moveit! configuration of SCARA
Run this launchfile for only kinematic simulation (in your local PC)
```
roslaunch scara_and_rt_moveit_config demo_rviz.launch
```
Run this launchfile for kinematic and dynamic simulation (in your local PC)
```
roslaunch scara_and_rt_moveit_config demo_gazebo.launch
```
Run this launchfile for simulation via Simulink (not really helpfull)
```
roslaunch scara_and_rt_moveit_config demo_matlab_simulink.launch
```
Run this launchfile for simulation via Matlab (PC-Matlab-SLRT-SCARA)  **Most important launchfile!**
```
roslaunch scara_and_rt_moveit_config demo_matlab_mfile.launch
```

## Part 3 - gazebo_scara_and_rt - Gazebo dynamic simulation
1. **scara_v2_full_description** : Rviz kinematic simulation
    - This is used to test the SCARA manipulator (such as part 1), but here are saved the URDF models for Gazebo.
     ```
        1. roslaunch scara_v2_full_description xacro_display_scara_rt.launch
        2. roslaunch scara_v2_full_description xacro_display_scara.launch
        3. roslaunch scara_v2_full_description xacro_display_rotary_table.launch
    ```
2. **scara_v2_full_gazebo** : Gazebo world for SCARA
    - Default world (working space) for the SCARA and RT. To run default world use one of the following commands:
    ```
        1. roslaunch scara_v2_full_gazebo scara_v2_full_world.launch
        2. roslaunch scara_v2_full_gazebo scara_and_rt_world.launch.launch
        3. roslaunch scara_v2_full_gazebo rt_v2_world.launch
    ```
3. **scara_v2_full_control** : Gazebo controllers
    - In this directory the controllers for SCARA and RT are defined. To launch it use the following commands:
    ```
        1. roslaunch scara_v2_full_control scara_v2_full_control.launch
        2. roslaunch scara_v2_full_control scara_and_rt_control.launch
        3. roslaunch scara_v2_full_control rt_v2_full_control.launch
    ```
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

## Part 5 - SCARA GUI
* 'TO DO - Add scripts'

## Part 6 - Rotary table:
1. **RViz**
```
roslaunch scara_v2_with_gripper xacro_display_rotary_table_v3.launch
```
2. **Moveit**
```
roslaunch rt_moveit_config demo.launch
```
3. **Gazebo**
```
roslaunch scara_v2_full_gazebo rt_v3_world.launch
roslaunch scara_v2_full_control rt_v3_full_control.launch
```
**Polohovanie**
```
rostopic pub -1 /rotary_table_v3/rt_position_controller/command std_msgs/Float64 "data: 1.57"
```
## Part 7 - Rotary Table GUI
#### Prerequisites
```
Install PEAK driver as shown in catkin_ws/src/1_Documentation/pcan_instalation.odt .
Then launch script "usb_to_can0"
    - cd /home/viktordluhos/catkin_ws/src/1_Documentation/
    - sudo ./usb_to_can0
    - if no error occures you can proceed to next steps
```
1. Connect the Rotary Table to 24V DC Source
2. Connect the PEAK to PCs USB and to Rotary Table
3. Switch on the power supply
4. **Start ROS Master**
```
roscore
```
5. **Start the Rotary Table menu**
```
rosrun scara_v2_moveit_api rotary_table_menu
```
6. **Start the Rotary Table GUI**
```
rosrun rotary_table_gui rotary_table_gui
```
Or you can ignore steps 4,5,6 and **use lanchfile**
```
roslaunch scara_v2_moveit_api rt_menu_and_gui_launch.launch
```

# TO DOs
- [ ] Part 5 - Add scripts
- [ ] Add comments to code




