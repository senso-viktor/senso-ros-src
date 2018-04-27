cmake_minimum_required(VERSION 2.8.3)
project(scara_v2_moveit_api)

add_compile_options(-std=c++11)

#find_package(Eigen3 REQUIRED)
find_package(PkgConfig)
pkg_search_module(Eigen3 REQUIRED eigen3)

find_package(catkin REQUIRED COMPONENTS
        moveit_core
        moveit_ros_planning_interface
        moveit_ros_planning
        moveit_msgs
        roscpp
        std_msgs
        sensor_msgs
#        map_msgs
        genmsg
        message_generation
        actionlib
        scara_msgs
        tf
        can_interface

)
find_package(Boost REQUIRED system filesystem date_time thread)



add_message_files(
        FILES
        pose_and_gripperState.msg
        pose_velocity_direction.msg
        status_rt.msg
)
 add_service_files(
   FILES
    SimpleService.srv
 )

generate_messages(
        DEPENDENCIES
        std_msgs
        sensor_msgs
#        map_msgs
        actionlib_msgs
        scara_msgs

)

catkin_package(
        CATKIN_DEPENDS
        moveit_core
        moveit_ros_planning_interface
        message_runtime
        std_msgs
        sensor_msgs
#        map_msgs

)


include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_executable(move_group_interface1 ../scara_v2_moveit_api/src/move_group_interface1.cpp)
target_link_libraries(move_group_interface1 ${catkin_LIBRARIES})

add_executable(move_group_interface2 ../scara_v2_moveit_api/src/move_group_interface2.cpp)
target_link_libraries(move_group_interface2 ${catkin_LIBRARIES})

add_executable(move_group_interface3_PICKandPLACE ../scara_v2_moveit_api/src/move_group_interface3_PICKandPLACE.cpp)
target_link_libraries(move_group_interface3_PICKandPLACE ${catkin_LIBRARIES})
#add_dependencies(move_group_interface3_PICKandPLACE beginner_tutorials_generate_messages_cpp)
add_dependencies(move_group_interface3_PICKandPLACE {${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(move_group_interface4_position_control ../scara_v2_moveit_api/src/move_group_interface4_position_control.cpp)
target_link_libraries(move_group_interface4_position_control ${catkin_LIBRARIES})
add_dependencies(move_group_interface4_position_control {${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(move_group_interface5_rt ../scara_v2_moveit_api/src/move_group_interface5_rt.cpp)
target_link_libraries(move_group_interface5_rt ${catkin_LIBRARIES})
add_dependencies(move_group_interface5_rt {${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(move_group_interface5_rt beginner_tutorials_gencpp)

add_executable(move_group_interface6_async_PICKandPLACE ../scara_v2_moveit_api/src/move_group_interface6_async_PICKandPLACE.cpp)
target_link_libraries(move_group_interface6_async_PICKandPLACE ${catkin_LIBRARIES})
add_dependencies(move_group_interface6_async_PICKandPLACE {${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(move_group_interface6_async_PICKandPLACE beginner_tutorials_gencpp)

add_executable(cube_publisher ../scara_v2_moveit_api/src/cube_publisher.cpp)
target_link_libraries(cube_publisher ${catkin_LIBRARIES})
add_dependencies(cube_publisher beginner_tutorials_generate_messages_cpp)

add_executable(multiple_cube_publisher ../scara_v2_moveit_api/src/multiple_cube_publisher.cpp)
target_link_libraries(multiple_cube_publisher ${catkin_LIBRARIES})
add_dependencies(multiple_cube_publisher beginner_tutorials_generate_messages_cpp)

add_executable(multiple_cylinder_publisher_scara_v3 ../scara_v2_moveit_api/src/multiple_cylinder_publisher_scara_v3.cpp)
target_link_libraries(multiple_cylinder_publisher_scara_v3 ${catkin_LIBRARIES})
add_dependencies(multiple_cylinder_publisher_scara_v3 beginner_tutorials_generate_messages_cpp)

add_executable(gripper_control ../scara_v2_moveit_api/src/gripper_control.cpp)
target_link_libraries(gripper_control ${catkin_LIBRARIES})
add_dependencies(gripper_control beginner_tutorials_generate_messages_cpp)

add_executable(matlab_communication_tryout ../scara_v2_moveit_api/src/matlab_communication_tryout.cpp)
target_link_libraries(matlab_communication_tryout ${catkin_LIBRARIES})

add_executable(matlab_ROS_com_tryout ../scara_v2_moveit_api/src/matlab_ROS_com_tryout.cpp)
target_link_libraries(matlab_ROS_com_tryout ${catkin_LIBRARIES})

add_executable(IK_test ../scara_v2_moveit_api/src/IK_test.cpp)
target_link_libraries(IK_test ${catkin_LIBRARIES})

add_executable(IK_test_real_scara ../scara_v2_moveit_api/src/IK_test_real_scara.cpp)
target_link_libraries(IK_test_real_scara ${catkin_LIBRARIES})

add_executable(real_scara_torq_sensor ../scara_v2_moveit_api/src/real_scara_torq_sensor.cpp)
target_link_libraries(real_scara_torq_sensor ${catkin_LIBRARIES})

add_executable(get_and_send_planned_path ../scara_v2_moveit_api/src/get_and_send_planned_path.cpp)
target_link_libraries(get_and_send_planned_path ${catkin_LIBRARIES})

add_executable(linear_interpolation ../scara_v2_moveit_api/src/linear_interpolation.cpp)
target_link_libraries(linear_interpolation ${catkin_LIBRARIES})

add_executable(tf_readout ../scara_v2_moveit_api/src/tf_readout.cpp)
target_link_libraries(tf_readout ${catkin_LIBRARIES})

add_executable(scara_menu ../scara_v2_moveit_api/src/scara_menu.cpp)
target_link_libraries(scara_menu ${catkin_LIBRARIES})

add_executable(scara_colision_object ../scara_v2_moveit_api/src/scara_colision_object.cpp)
target_link_libraries(scara_colision_object ${catkin_LIBRARIES})

add_executable(rotary_table_menu ../scara_v2_moveit_api/src/rotary_table_menu.cpp)
target_link_libraries(rotary_table_menu ${catkin_LIBRARIES})


