cmake_minimum_required(VERSION 2.8.3)
project(denso_state_behavior)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  denso_state_srvs
  moveit_core
  moveit_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  roscpp
)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS
  denso_state_srvs
  moveit_core
  moveit_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  roscpp
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(moving_behavior
  src/moving/moving_behavior.cpp
  src/moving/moving_behavior_node.cpp)
add_dependencies(moving_behavior
  denso_state_msgs_generate_messages_cpp)
add_dependencies(
  moving_behavior
  denso_state_srvs)
target_link_libraries(moving_behavior
  ${catkin_LIBRARIES})

###########
# Install #
###########

set(EXEC_FILES
  moving_behavior)

set(DIRECTORIES
  launch)

# Install executables
foreach(nodes ${EXEC_FILES})
  install(TARGETS ${nodes}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endforeach()

# Install include directory
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# Install all files
foreach(dirs ${DIRECTORIES})
  install(DIRECTORY ${dirs}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dirs})
endforeach()

#############
