cmake_minimum_required(VERSION 2.8.3)
project(using_image_pipeline)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  image_transport
  nodelet
  pcl_conversions
  pcl_ros
  ddynamic_reconfigure
)
# find_package(PCL 1.2 REQUIRED)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-Wno-deprecated-declarations")

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES using_image_pipeline
#  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})

add_executable(image_shrinker src/image_shrinker.cpp)
add_dependencies(image_shrinker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(image_shrinker  ${DEPENDENCIES} ${catkin_LIBRARIES})

add_library(${PROJECT_NAME} src/nodelets/pcl_segmentation.cpp src/nodelets/transform_cloud.cpp)
# add_dependencies(${PROJECT_NAME} )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# add_executable(pcl_segmentation_node src/pcl_segmentation_node.cpp)
# add_dependencies(pcl_segmentation_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(pcl_segmentation_node  ${DEPENDENCIES} ${catkin_LIBRARIES})
