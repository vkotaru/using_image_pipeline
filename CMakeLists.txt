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
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES using_image_pipeline
#  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(image_shrinker src/main.cpp)
add_dependencies(image_shrinker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(image_shrinker  ${DEPENDENCIES} ${catkin_LIBRARIES})
