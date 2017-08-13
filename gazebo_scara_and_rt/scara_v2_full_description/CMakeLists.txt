cmake_minimum_required(VERSION 2.8.3)
project(scara_v2_full_description)

find_package(catkin REQUIRED COMPONENTS
        joint_state_publisher
        robot_state_publisher
        rviz
        )

catkin_package()

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY meshes
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY urdf
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

include_directories(
        ${catkin_INCLUDE_DIRS}
)