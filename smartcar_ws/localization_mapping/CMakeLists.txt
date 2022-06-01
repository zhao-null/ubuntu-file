cmake_minimum_required(VERSION 2.8.3)
project(localization_mapping)

SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
add_compile_options(-std=c++11)
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  geometry_msgs
  message_generation
  pcl_ros
  roscpp
  rospy
  std_msgs
  std_srvs
  tf
  cv_bridge
  message_generation
)

find_package(OpenCV 3 QUIET)


include(cmake/YAML.cmake)
include(cmake/PCL.cmake)
include(cmake/eigen.cmake)
#include(cmake/geographic.cmake)
include(cmake/g2o.cmake)

#include_directories(include ${catkin_INCLUDE_DIRS})
#include(cmake/global_defination.cmake)
add_message_files(FILES range_image.msg point_range_image.msg)
generate_messages(DEPENDENCIES std_msgs sensor_msgs)

catkin_package(
 #   INCLUDE_DIRS include
    CATKIN_DEPENDS message_runtime
)

file(GLOB_RECURSE ALL_SRCS "*.cpp")
file(GLOB_RECURSE NODE_SRCS "src/apps/*_node.cpp")
file(GLOB_RECURSE THIRD_PARTY_SRCS "third_party/*.cpp")
list(REMOVE_ITEM ALL_SRCS ${NODE_SRCS})
list(REMOVE_ITEM ALL_SRCS ${THIRD_PARTY_SRCS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(data_pretreat_node src/apps/data_pretreat_node.cpp ${ALL_SRCS})
add_dependencies(data_pretreat_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(data_pretreat_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES} ${OpenCV_LIBS})
