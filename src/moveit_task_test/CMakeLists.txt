cmake_minimum_required(VERSION 3.0.2)
project(moveit_task_test)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_task_constructor_core
  moveit_task_constructor_msgs
  roscpp
  rospy
  std_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES moveit_task_test
#  CATKIN_DEPENDS moveit_core moveit_ros_planning moveit_ros_planning_interface moveit_task_constructor_core moveit_task_constructor_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/moveit_task_test.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/moveit_task_test_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/move_ur5.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
# catkin_install_python(PROGRAMS
#   scripts/aruco_subscriber.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
# catkin_install_python(PROGRAMS
#   scripts/move_to_node.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
catkin_install_python(PROGRAMS
  scripts/home_move.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
# catkin_install_python(PROGRAMS
#   scripts/gripper.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
catkin_install_python(PROGRAMS
  scripts/follow_marker.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
# catkin_install_python(PROGRAMS
#   scripts/follow_marker_2.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
# catkin_install_python(PROGRAMS
#   scripts/follow_marker_3.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
# catkin_install_python(PROGRAMS
#   scripts/follow_marker_4.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
# catkin_install_python(PROGRAMS
#   scripts/follow_marker_5.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
# # catkin_install_python(PROGRAMS
#   scripts/follow_marker_6.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
# catkin_install_python(PROGRAMS
#   scripts/servo_1.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
# catkin_install_python(PROGRAMS
#   scripts/get_pose.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
catkin_install_python(PROGRAMS
  scripts/docking_position.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_moveit_task_test.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
