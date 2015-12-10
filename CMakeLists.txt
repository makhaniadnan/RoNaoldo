cmake_minimum_required(VERSION 2.8.3)
project(RoNAOldo)


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

add_executable(RoNAOldo_vision src/vision.cpp)
target_link_libraries(RoNAOldo_vision ${catkin_LIBRARIES})
add_dependencies(RoNAOldo_vision RoNAOldo_generate_message_cpp)

add_executable(RoNAOldo_control src/control.cpp)
target_link_libraries(RoNAOldo_control ${catkin_LIBRARIES})
add_dependencies(RoNAOldo_control RoNAOldo_generate_message_cpp)

find_package(Eigen REQUIRED)
target_link_libraries(RoNAOldo_vision ${Eigen_LIBRARIES})
target_link_libraries(RoNAOldo_control ${Eigen_LIBRARIES})

find_package(Boost REQUIRED COMPONENTS system thread)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(RoNAOldo_vision ${Boost_LIBRARIES})
target_link_libraries(RoNAOldo_control ${Boost_LIBRARIES})

find_package(OpenCV REQUIRED)
target_link_libraries(RoNAOldo_vision ${OpenCV_LIBS})
target_link_libraries(RoNAOldo_control ${OpenCV_LIBS})

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} /usr/local/lib/cmake)

find_package(aruco REQUIRED)
target_link_libraries(RoNAOldo_vision  ${aruco_LIBS})
target_link_libraries(RoNAOldo_control  ${aruco_LIBS})

