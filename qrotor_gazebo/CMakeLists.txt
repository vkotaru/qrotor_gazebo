cmake_minimum_required(VERSION 3.0.2)
project(qrotor_gazebo)
find_package(gazebo)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        geometry_msgs
        roscpp
        rospy
        std_msgs
        roscpp
        gazebo_plugins
        gazebo_ros
        sensor_msgs
        tf2
        tf2_ros
        tf2_geometry_msgs
        eigen_conversions
        message_generation
        )

add_message_files(
        FILES
        Spline.msg
        Command.msg
)

generate_messages(
        DEPENDENCIES geometry_msgs std_msgs
)

catkin_package(
          INCLUDE_DIRS include
        #  LIBRARIES qrotor_gazebo
        CATKIN_DEPENDS roscpp gazebo_plugins gazebo_ros geometry_msgs std_msgs tf2 tf2_ros sensor_msgs message_runtime
        DEPENDS EIGEN3 GAZEBO
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${Eigen_INCLUDE_DIRS}
        ${GAZEBO_INCLUDE_DIRS}
        ${SDFormat_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(qrotor_plugin SHARED src/qrotor_plugin.cpp)
target_link_libraries(qrotor_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(qrotor_plugin ${catkin_EXPORTED_TARGETS})