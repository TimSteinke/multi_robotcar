cmake_minimum_required(VERSION 3.15.0)
project(radar_robotcar_player)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  message_generation
  tf
  cv_bridge
  image_transport
  pcl_ros
  pcl_conversions
)
find_package(OpenCV REQUIRED)
find_package(Eigen3)
find_package(PCL REQUIRED)

add_message_files(FILES
        OxfordTime.msg
        )

generate_messages(DEPENDENCIES std_msgs)

set_target_properties(${TARGET} PROPERTIES
COMPILE_FLAGS "-pthread"
LINK_FLAGS "-pthread")

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  )

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS std_msgs roscpp message_runtime
  DEPENDS EIGEN3 OpenCV PCL
)

add_executable(${PROJECT_NAME}_node src/radar_robotcar_player_node.cpp src/radar_robotcar_player.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${CERES_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}
  ${PCL_LIBRARIES}
  )
