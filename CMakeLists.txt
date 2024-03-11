 # Copyright (C) 2024 David Skuddis - All Rights Reserved
 # You may use, distribute and modify this code under the
 # terms of the MIT license.
 #
 # You should have received a copy of the MIT license with
 # this file. If not, please write to: davidskuddis@web.de, or visit: https://opensource.org/license/mit/



cmake_minimum_required(VERSION 3.0.2)
project(dense_cloud_creator)

set(CMAKE_BUILD_TYPE release)

set(CMAKE_CXX_FLAGS_RELEASE "-O1")

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  rosbag
  sensor_msgs
  roscpp
  rospy
  std_msgs
  geometry_msgs
  tf
  pcl_conversions)

find_package(Boost 1.40.0 COMPONENTS filesystem system iostreams REQUIRED)

find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})

find_package(PCL 1.3 REQUIRED COMPONENTS)

catkin_package(
    CATKIN_DEPENDS roscpp rospy std_msgs tf pcl_conversions sensor_msgs geometry_msgs rosbag
)

include_directories(
        include
        Thirdparty
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

add_executable(dense_cloud_creator
               src/main.cpp
               src/dense_cloud_creator.cpp
               )


target_link_libraries(dense_cloud_creator ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES})
