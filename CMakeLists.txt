cmake_minimum_required(VERSION 2.8.3)
project(geo_tagger)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  image_transport
  cv_bridge
  message_generation
)

find_package(Exiv2)
find_package( Boost REQUIRED COMPONENTS
  system
  filesystem
)

add_definitions(-std=c++11)

catkin_package( CATKIN_DEPENDS
  roscpp
  roslib
  message_runtime
        std_msgs sensor_msgs
)

add_message_files(
        FILES
        GeoImageCompressed.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
        sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Exiv2_INCLUDE_DIRS}
)

## For Flight Node
add_executable(geo_tagger_node
  src/geo_tagger_node.cpp
)

add_dependencies(geo_tagger_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  
)

target_link_libraries(geo_tagger_node
  ${catkin_LIBRARIES}
  ${Exiv2_LIBRARIES}
  -lexiv2
)





