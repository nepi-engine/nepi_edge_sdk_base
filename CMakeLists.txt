#[[
Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.

This file is part of nepi-engine
(see https://github.com/nepi-engine).

License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
]]
cmake_minimum_required(VERSION 2.8.3)
project(nepi_edge_sdk_base)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)
add_compile_options(-Wall)

# Following relies (sometimes, e.g., Jetpack-installed OpenCV) on a symlink from /usr/share/OpenCV to
# true OpenCV cmake folder (e.g., usr/lib/aarch64-linux-gnu/cmake/opencv4 on Jetson)
set(OpenCV_DIR /usr/share/OpenCV)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
## Note that using COMPONENTS here ensures that the package environment variables
## are combined into the generic corresponding catkin_<ENV_VARIABLE> stuff, so, for example
## we don't need to explicitly link OpenCV_LIBRARIES, instead just link to catkin_LIBRARIES and
## get OpenCV for free (same holds for catkin_INCLUDE_DIRS, etc.)
find_package(
    catkin
    REQUIRED
    COMPONENTS
    message_generation
    nepi_ros_interfaces
    roscpp
    image_transport
    cv_bridge
    OpenCV
    sensor_msgs
    rospy
    std_msgs
    pcl_ros
    tf2_ros
    nav_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Use the verbose version of make
#set(CMAKE_VERBOSE_MAKEFILE ON)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   WakeUpEvent.msg
   NodeOutput.msg
)


## Generate services in the 'srv' folder
add_service_files(
   FILES
   NepiDataDir.srv
   ProcessCfgUpdates.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
     DEPENDENCIES
     std_msgs  # Or other packages containing msgs
     sensor_msgs
     nepi_ros_interfaces
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include 
    #include/drivers/generic_ahrs
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS image_transport nepi_ros_interfaces

#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
   include
   #include/drivers/generic_ahrs
   ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
   src/sdk_node.cpp
   src/sdk_interface.cpp
   src/sdk_utils.cpp
   #src/node_3dx.cpp
   src/trigger_interface.cpp
   src/net_trigger_interface.cpp
   src/save_data_interface.cpp
   src/trigger_mgr_base.cpp
   src/drivers/i2c.cpp
   src/drivers/ltc1669_dac.cpp
   src/drivers/vca8617.cpp
   src/drivers/ad9249_adc.cpp
   #src/drivers/generic_ahrs/ahrs_driver.cpp
   #src/drivers/generic_ahrs/lord_ahrs_driver.cpp
   #src/drivers/generic_ahrs/ros_ahrs_driver.cpp
   src/algorithms/approx_moving_avg.cpp
   src/algorithms/underwater_img_enhancer.cpp
)

## Lord AHRS ROS Wrapper Node
#add_executable(lord_ahrs_ros_wrapper
#  src/lord_ahrs_ros_wrapper)

#add_dependencies(lord_ahrs_ros_wrapper
#  ${catkin_EXPORTED_TARGETS}
#  ${PROJECT_NAME})

#target_link_libraries(lord_ahrs_ros_wrapper
#  ${PROJECT_NAME}
#  ${catkin_LIBRARIES})

## Trigger Manager (No hardware support) Node
add_executable(trigger_mgr_no_hardware
  src/trigger_mgr_no_hardware)

add_dependencies(trigger_mgr_no_hardware
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME})

target_link_libraries(trigger_mgr_no_hardware
  ${PROJECT_NAME}
  ${catkin_LIBRARIES})

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
#add_executable(<node_name>
#	<src/<node_name>.cpp
#)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
#add_dependencies(<node_name> ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
#target_link_libraries(<node_name>
#	${catkin_LIBRARIES}
#	${PROJECT_NAME}
#)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

catkin_install_python(PROGRAMS
  scripts/system_mgr.py
  scripts/config_mgr.py
  scripts/time_sync_mgr.py
  scripts/network_mgr.py
  scripts/image_color_converter.py
  scripts/rui_config_mgr.py
  scripts/nepi_autolauncher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#install(TARGETS lord_ahrs_ros_wrapper
#  lord_ahrs_ros_wrapper
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

install(TARGETS trigger_mgr_no_hardware
  trigger_mgr_no_hardware
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation - Msgs and Services are under the project name subdir
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

## Non-generated headers are located directly in include and get copied to the root of include in the install directory
install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  launch
  #urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# sys_env.bash goes in parent of the install folder
SET(SYS_ENV_DIR ${CMAKE_INSTALL_PREFIX}/..)
SET(SYS_ENV_FILE ${SYS_ENV_DIR}/sys_env.bash)

# Developer convenience to create a backup sys_env file before overwriting with the non-functional one in the repo.
install(FILES ${SYS_ENV_FILE}
  DESTINATION ${SYS_ENV_DIR}
  RENAME sys_env.bash.bak
  PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ GROUP_WRITE WORLD_READ
  OPTIONAL)

# Install etc files (startup scripts, factory configs, etc.) to the global (not project) etc destination
install(DIRECTORY etc/
  DESTINATION ${CATKIN_GLOBAL_ETC_DESTINATION}
  USE_SOURCE_PERMISSIONS
  PATTERN "sys_env.bash" EXCLUDE
)

# Install sys_env file in case it doesn't yet exist
install(FILES etc/sys_env.bash
  DESTINATION ${SYS_ENV_DIR}
  PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ GROUP_WRITE WORLD_READ
)

# Developer convenience to restore a backup sys_env file if one exists
install(FILES ${SYS_ENV_DIR}/sys_env.bash.bak
  DESTINATION ${SYS_ENV_DIR}
  RENAME sys_env.bash
  PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ GROUP_WRITE WORLD_READ
  OPTIONAL)

install(DIRECTORY utilities/
  DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)



#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_sdk.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
