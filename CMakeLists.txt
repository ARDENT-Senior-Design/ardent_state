cmake_minimum_required(VERSION 2.8.3)
project(ardent_state_simple)

if(CMAKE_COMPILER_IS_GNUCXX OR ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang"))
    set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  ardent_msgs_simple
  ardent_description_simple
  hardware_interface
  trac_ik_lib
  kdl_parser
  urdf
)

find_package(orocos_kdl REQUIRED)
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(TinyXML REQUIRED)  # parse xml
find_package(urdfdom REQUIRED)  # parse urdf 
# find_package(Boost REQUIRED COMPONENTS system signals thread)

include_directories(include ${catkin_INCLUDE_DIRS} 
                            # ${Boost_INCLUDE_DIRS} 
                            ${orocos_kdl_INCLUDE_DIRS}
                            ${EIGEN3_INCLUDE_DIRS}
                            )


add_library(${PROJECT_NAME} 
  src/leg_kinematic_tree.cpp
  src/robot.cpp
  )

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
    sensor_msgs
    # gazebo_ros
    hardware_interface
    urdf
    trac_ik_lib
    ardent_description_simple
  DEPENDS 
    Boost
    orocos_kdl
    TinyXML
  LIBRARIES ${PROJECT_NAME}
)

# add_executable(walking_test test/walking_test.cpp)
# target_link_libraries(walking_test ${catkin_LIBRARIES} ${PROJECT_NAME})
add_executable(leg_state_control scripts/leg_state_control.cpp)
target_link_libraries(leg_state_control ${catkin_LIBRARIES} ${PROJECT_NAME} ${orocos_kdl_LIBRARIES})
# add_executable(single_leg_test_raw test/single_leg_test_raw.cpp)
# target_link_libraries(single_leg_test_raw ${catkin_LIBRARIES} ${PROJECT_NAME})
# add_executable(thread_test test/thread_test.cpp)
# target_link_libraries(thread_test ${catkin_LIBRARIES} ${PROJECT_NAME})


install(DIRECTORY include/${PROJECT_NAME}/
DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})


install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


