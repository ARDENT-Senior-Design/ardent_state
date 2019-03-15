cmake_minimum_required(VERSION 2.8.3)
project(ardent_state_simple)


add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  ardent_msgs_simple
  ardent_description_simple
  hardware_interface
  kdl_parser
  trac_ik_lib
  urdf
)
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(TinyXML REQUIRED)  # parse xml
find_package(urdfdom REQUIRED)  # parse urdf 
find_package(Boost REQUIRED COMPONENTS system signals thread)


include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})


add_library(${PROJECT_NAME} 
  src/kinematic_tree.cpp
  )

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
    sensor_msgs
    # gazebo_ros
    hardware_interface
    urdf
    ardent_description_simple
  DEPENDS 
    Boost
    TinyXML
  LIBRARIES ${PROJECT_NAME}
)

add_executable(walking_test test/walking_test.cpp)
target_link_libraries(walking_test ${catkin_LIBRARIES} ${PROJECT_NAME})
add_executable(single_leg_test test/single_leg_test.cpp)
target_link_libraries(single_leg_test ${catkin_LIBRARIES} ${PROJECT_NAME})


target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${catkin_LIBRARIES})


install(DIRECTORY include/${PROJECT_NAME}/
DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})


install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


