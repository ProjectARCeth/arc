cmake_minimum_required(VERSION 2.8.3)
project(arc)

find_package(catkin_simple 0.1.0 REQUIRED COMPONENTS
  	roscpp
    roslib
    rospy
    geometry_msgs
    nav_msgs
    sensor_msgs
    std_msgs
    eigen_catkin
    pcl_conversions
    pcl_ros
    catkin_simple REQUIRED
    tf
)

include_directories(${Qt5Widgets_INCLUDE_DIRS})

catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(${PROJECT_NAME} 
    src/CarModel.cpp
    src/GridAnalyser.cpp
    src/Guard.cpp
    src/ObstacleDetection.cpp
    src/Information_and_Tools.cpp
    src/PurePursuit.cpp
    src/ROSInterface.cpp
    src/StateEstimation.cpp
    src/VCUInterface.cpp  
)

cs_add_executable(main src/main.cpp)
target_link_libraries(main ${catkin_LIBRARIES} ${PROJECT_NAME})

cs_install()
cs_export()