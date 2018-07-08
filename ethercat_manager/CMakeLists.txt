cmake_minimum_required(VERSION 2.8.3)
project(ethercat_manager)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  soem
)


catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES ethercat_manager
  CATKIN_DEPENDS roscpp

)


include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${soem_INCLUDE_DIRS}/soem
)

## Declare a C++ library
add_library(ethercat_manager src/ethercat_manager.cpp)
target_link_libraries(ethercat_manager ${catkin_LIBRARIES})


## Declare a C++ executable
add_executable(slaveInfo src/tests/slaveInfo.cpp)
target_link_libraries(slaveInfo ${catkin_LIBRARIES})

add_executable(simple_test_velocity src/tests/simple_test_velocity.cpp)
target_link_libraries(simple_test_velocity ethercat_manager ${catkin_LIBRARIES})



## Mark executables and/or libraries for installation
install(TARGETS ethercat_manager
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )


