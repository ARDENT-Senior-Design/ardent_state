cmake_minimum_required(VERSION 2.8.3)
project(ardent_state)


add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  moveit_core
  moveit_ros_planning 
  moveit_ros_planning_interface 
  ardent_msgs
  ardent_description
  hardware_interface
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
  src/robot.cpp
  src/joint.cpp  
  )

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
    sensor_msgs
    # gazebo_ros
    hardware_interface
    urdf
    ardent_description
  DEPENDS 
    Boost
    TinyXML
  LIBRARIES ${PROJECT_NAME}
)

# add_executable(walking_test test/walking_test.cc)
# target_link_libraries(walking_test ${catkin_LIBRARIES} ${PROJECT_NAME})
add_executable(moveit_state_test test/moveit_state_test.cpp)
target_link_libraries(moveit_state_test ${catkin_LIBRARIES})

target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${catkin_LIBRARIES})


install(DIRECTORY include/${PROJECT_NAME}/
DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})


install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


