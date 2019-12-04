cmake_minimum_required(VERSION 2.8.3)
project(Path_planning)

#set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
add_compile_options(-std=c++14)


find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  geometry_msgs
  moveit_commander
  moveit_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  pluginlib
  roscpp
  tf
  moveit_visual_tools
)





find_package(Boost REQUIRED system filesystem date_time thread)
find_package(Eigen3 REQUIRED)

include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

catkin_package(
  CATKIN_DEPENDS
  moveit_core
  moveit_ros_planning_interface
  )

add_executable(dataset src/dataset.cpp)
target_link_libraries(dataset ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(moving_example src/moving_example.cpp)
target_link_libraries(moving_example ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(ik_solve src/ik_solve.cpp)
target_link_libraries(ik_solve ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(ik_test src/ik_test.cpp)
target_link_libraries(ik_test ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(ik_stomp src/ik_stomp.cpp)
target_link_libraries(ik_stomp ${catkin_LIBRARIES} ${Boost_LIBRARIES})
