cmake_minimum_required(VERSION 2.8.3)
project(scara_msgs)


find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        genmsg
        message_generation
        )
find_package(Boost REQUIRED system filesystem date_time thread)


add_message_files(
        FILES
        pose_and_gripperState.msg
#        pose_and_gripperState1.msg
        robot_info.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
)

catkin_package(
        CATKIN_DEPENDS
        message_runtime
        std_msgs
)
