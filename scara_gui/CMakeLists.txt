cmake_minimum_required(VERSION 2.8.3)
project(scara_gui)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rqt_gui
        rqt_gui_cpp
        std_msgs
        scara_msgs
)
find_package(Qt5Widgets REQUIRED)

set(${PROJECT_NAME}_cpp_HDRS include/scara_gui/mainwindow.h)
set(${PROJECT_NAME}_cpp_SRCS src/mainwindow.cpp src/main.cpp)
set(${PROJECT_NAME}_cpp_UIS  user_interface/mainwindow.ui)

qt5_wrap_cpp(${PROJECT_NAME}_cpp_MOCS ${${PROJECT_NAME}_cpp_HDRS})
qt5_wrap_ui(${PROJECT_NAME}_cpp_UIS_H ${${PROJECT_NAME}_cpp_UIS})


generate_messages(
        DEPENDENCIES
        std_msgs
        scara_msgs
)


catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp rqt_gui rqt_gui_cpp std_msgs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}
    include
)

add_executable(${PROJECT_NAME}
        ${${PROJECT_NAME}_cpp_HDRS}
        ${${PROJECT_NAME}_cpp_SRCS}
        ${${PROJECT_NAME}_cpp_MOCS}
        ${${PROJECT_NAME}_cpp_UIS_H})

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        #${QT_QTCORE_LIBRARY}
        #${QT_QTGUI_LIBRARY}
        Qt5::Widgets
        )

