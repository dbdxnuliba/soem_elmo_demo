cmake_minimum_required(VERSION 2.8.3)
project(elmo_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  diagnostic_updater
  ethercat_manager
  roscpp
  sensor_msgs
)

find_package (Threads)


catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES elmo_client elmo_library
  CATKIN_DEPENDS roscpp diagnostic_updater sensor_msgs
)



include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(elmo_client src/elmo_client.cpp)
add_library(elmo_library src/elmo_library.cpp)

target_link_libraries(elmo_client ${catkin_LIBRARIES})
target_link_libraries(elmo_library ${catkin_LIBRARIES} elmo_client)

install(TARGETS elmo_client elmo_library
 ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}

)


## Declare a C++ executable
add_executable(elmo_client_velocity_control src/tests/elmo_client_velocity_control.cpp)
target_link_libraries(elmo_client_velocity_control ${catkin_LIBRARIES} elmo_client)

add_executable(elmo_client_velocity_control_teleop src/tests/elmo_client_velocity_control_teleop.cpp)
target_link_libraries(elmo_client_velocity_control_teleop ${catkin_LIBRARIES} elmo_client)

add_executable(elmo_library_velocity_control src/tests/elmo_library_velocity_control.cpp)
target_link_libraries(elmo_library_velocity_control ${catkin_LIBRARIES} elmo_library)



