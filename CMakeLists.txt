cmake_minimum_required(VERSION 2.8.3)
project(RoNAOldo)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED cmake_modules COMPONENTS
  roscpp
  std_msgs
  naoqi_bridge_msgs
  std_srvs
  tf
  actionlib_msgs
  cv_bridge
  image_transport
  message_generation
)

add_message_files(
  FILES
  visionMsg.msg
  controlMsg.msg
  goalPositionMsg.msg
  ballMsg.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
add_executable(RoNAOldo_ballVision src/visionBall.cpp)
target_link_libraries(RoNAOldo_ballVision ${catkin_LIBRARIES})
add_dependencies(RoNAOldo_ballVision RoNAOldo_generate_message_cpp)

add_executable(RoNAOldo_dummyVision src/dummyVision.cpp)
target_link_libraries(RoNAOldo_dummyVision ${catkin_LIBRARIES})
add_dependencies(RoNAOldo_dummyVision RoNAOldo_generate_message_cpp)


add_executable(RoNAOldo_vision_marker src/visionMarker.cpp)
target_link_libraries(RoNAOldo_vision_marker ${catkin_LIBRARIES})
add_dependencies(RoNAOldo_vision_marker RoNAOldo_generate_message_cpp)

add_executable(RoNAOldo_control src/control.cpp)
target_link_libraries(RoNAOldo_control ${catkin_LIBRARIES})
add_dependencies(RoNAOldo_control RoNAOldo_generate_message_cpp)


add_executable(RoNAOldo_relative_position src/relativePosition.cpp)
target_link_libraries(RoNAOldo_relative_position ${catkin_LIBRARIES})
add_dependencies(RoNAOldo_relative_position RoNAOldo_generate_message_cpp)

find_package(Eigen REQUIRED)
target_link_libraries(RoNAOldo_ballVision ${Eigen_LIBRARIES})
target_link_libraries(RoNAOldo_relative_position ${Eigen_LIBRARIES})
target_link_libraries(RoNAOldo_control ${Eigen_LIBRARIES})
target_link_libraries(RoNAOldo_dummyVision ${Eigen_LIBRARIES})

find_package(Boost REQUIRED COMPONENTS system thread)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(RoNAOldo_ballVision ${Boost_LIBRARIES})
target_link_libraries(RoNAOldo_dummyVision ${Boost_LIBRARIES})
target_link_libraries(RoNAOldo_control ${Boost_LIBRARIES})

find_package(OpenCV REQUIRED)
target_link_libraries(RoNAOldo_ballVision ${OpenCV_LIBS})
target_link_libraries(RoNAOldo_dummyVision ${OpenCV_LIBS})
target_link_libraries(RoNAOldo_control ${OpenCV_LIBS})

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} /usr/local/lib/cmake)

find_package(aruco REQUIRED)
target_link_libraries(RoNAOldo_ballVision  ${aruco_LIBS})
target_link_libraries(RoNAOldo_dummyVision  ${aruco_LIBS})
target_link_libraries(RoNAOldo_vision_marker  ${aruco_LIBS})
target_link_libraries(RoNAOldo_control  ${aruco_LIBS})
