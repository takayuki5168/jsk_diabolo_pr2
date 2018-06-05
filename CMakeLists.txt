cmake_minimum_required(VERSION 2.8.3)
project(jsk_diabolo_pr2)

find_package(catkin REQUIRED COMPONENTS
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

add_executable(calc_idle_diabolo_state src/calc_idle_diabolo_state.cpp)
target_link_libraries(calc_idle_diabolo_state ${catkin_LIBRARIES})

add_executable(calc_toss_diabolo_pos src/calc_toss_diabolo_pos.cpp)
target_link_libraries(calc_toss_diabolo_pos ${catkin_LIBRARIES})

add_executable(publish_diabolo_model src/publish_diabolo_model.cpp)
target_link_libraries(publish_diabolo_model ${catkin_LIBRARIES})

#add_executable(toss_diabolo_marker src/toss_diabolo_marker.cpp)
#target_link_libraries(toss_diabolo_marker ${catkin_LIBRARIES})

#add_executable(particle_system src/particle_system.cpp)
#target_link_libraries(particle_system ${catkin_LIBRARIES})
