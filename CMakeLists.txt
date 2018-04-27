
cmake_minimum_required(VERSION 2.8.3)
project(pr2_juggle)

find_package(catkin REQUIRED COMPONENTS
  # fetcheus
  jsk_maps
  pr2eus
  sensor_msgs
  roscpp
  pcl_conversions
  pcl_ros
)

find_package(Boost REQUIRED COMPONENTS system)

# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES jsk_2017_10_semi
#  CATKIN_DEPENDS fetcheus jsk_maps pr2eus
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(sample_pcl src/sample_pcl.cpp)
target_link_libraries(sample_pcl ${catkin_LIBRARIES})

add_executable(diabolo_marker src/diabolo_marker.cpp)
target_link_libraries(diabolo_marker ${catkin_LIBRARIES})
