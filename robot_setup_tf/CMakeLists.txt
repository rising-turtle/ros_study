cmake_minimum_required(VERSION 2.8.3)
project(robot_setup_tf)

if(UNIX)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
    #SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++0x")
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  tf
  std_msgs
  cv_bridge
  tf_conversions
)

find_package(OpenCV REQUIRED)
# find_package(PCL 1.7.1 EXACT REQUIRED)

# PCL #######################
 set(PCL_DIR "/home/davidz/work/3rdlibs/pcl_1.7.1_src/pcl-pcl-1.7.1/build")
 find_package(PCL 1.7 REQUIRED COMPONENTS common io)
 include_directories(${PCL_INCLUDE_DIRS})
 link_directories(${PCL_LIBRARY_DIRS})
 # add_definitions(${PCL_DEFINITIONS})



## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

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
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs
# )

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
## Declare things tfind_package(PCL 1.3 REQUIRED COMPONENTS common io)o be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot_setup_tf
#  CATKIN_DEPENDS geometry_msgs roscpp tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${OpenCV_INCLUDE_DIRS}        # /usr/include/opencv;/usr/include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  # /home/emaad22/vnccpplib-0.3.2/include;/home/emaad22/vnccpplib-0.3.2/src;/home/emaad22/vnccpplib-0.3.2/src/arch/linux;
  /usr/local/MATLAB/R2014a/extern/include
)
#message("OpenCV_libs: ${OpenCV_LIBRARIES}") #just to check of this variable is populated correctly
#message("OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}") #just to check of this variable is populated correctly

## Declare a C++ library
# add_library(robot_setup_tf
#   src/${PROJECT_NAME}/robot_setup_tf.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(robot_setup_tf ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(robot_setup_tf_node src/robot_setup_tf_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(robot_setup_tf_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(robot_setup_tf_node
#   ${catkin_LIBRARIES}
# )

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

## Mark executables and/or libraries for installation
# install(TARGETS robot_setup_tf robot_setup_tf_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_robot_setup_tf.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
set(matlab_LIBRARIES 	"/usr/local/MATLAB/R2014a/bin/glnxa64/libeng.so;/usr/local/MATLAB/R2014a/bin/glnxa64/libmx.so;/usr/lib/x86_64-linux-gnu/libtiff.so;/usr/lib/x86_64-linux-gnu/libfreetype.so")#;/lib/x86_64-linux-gnu;/lib64")

MESSAGE( STATUS "***********matlab_LIBRARIES:" ${matlab_LIBRARIES} )
#MESSAGE( STATUS "***********OpenCV_LIBRARIES:" ${OpenCV_LIBRARIES} )
# add_library(vn100 /home/emaad22/vnccpplib-0.3.2/src/vn100.c)
# add_library(vndevice /home/emaad22/vnccpplib-0.3.2/src/vndevice.c)
# add_library(vncp_services /home/emaad22/vnccpplib-0.3.2/src/arch/linux/vncp_services.c)

# add_executable(tf_broadcaster src/tf_broadcaster.cpp)
# add_executable(tf_listener src/tf_listener.cpp)
# add_executable(point_cloud_publisher src/point_cloud_publisher.cpp)
#add_executable(vo_publisher src/vo_publisher.cpp)
add_executable(ekf_subscriber src/ekf_subscriber.cpp)
# add_executable(rpekf_processor src/rpekf_processor.cpp)

# target_link_libraries(tf_broadcaster ${catkin_LIBRARIES})
# target_link_libraries(tf_listener ${catkin_LIBRARIES})
# target_link_libraries(point_cloud_publisher ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
#target_link_libraries(vo_publisher  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} vn100 vndevice vncp_services)
target_link_libraries(ekf_subscriber ${catkin_LIBRARIES} ${tf_conversions_LIBS} ${matlab_LIBRARIES} ${PCL_LIBRARIES})
# target_link_libraries(rpekf_processor ${catkin_LIBRARIES} ${tf_conversions_LIBS} ${OpenCV_LIBRARIES})

