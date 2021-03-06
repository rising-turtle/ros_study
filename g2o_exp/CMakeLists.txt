cmake_minimum_required(VERSION 2.4.6)
project(g2o_exp)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
)

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
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
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
#   std_msgs  # Or other packages containing msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
# catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES g2o_exp
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
# )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES my_pcl_tutorial
  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs 
  DEPENDS message_runtime std_msgs 
)


SET(CMAKE_CXX_FLAGS "-ggdb -O3 -fPIC -std=c++0x")

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})
# link_libraries(rgbdslam ${catkin_LIBRARIES})

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake-modules/")

## suitesparse 
include_directories(/usr/include/suitesparse/)

# for linking to libcxsparse2.2.3
link_directories("/usr/lib/i386-linux-gnu")

#############################
# Eigen ####################
#############################
# find_package(Eigen)
set(EIGEN_INCLUDE_DIRS "/usr/include/eigen3")
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

# g2o #######################
set(G2O_ROOT "/home/davidz/work/3rdlibs/g2o/install")
# set(G2O_INCLUDE_DIR "${G2O_DIR}/include")
# set(G2O_LIBRARIES "${G2O_DIR}/lib")
find_package(G2O REQUIRED)
set(G2O_INCLUDE_DIRS "${G2O_ROOT}/include")
include_directories(${G2O_INCLUDE_DIRS})
message(STATUS "G2O_INCLUDE_DIRS : ${G2O_INCLUDE_DIRS}")
set(G2O_LIBRARY_DIRS "${G2O_ROOT}/lib")
link_directories(${G2O_LIBRARY_DIRS})
# g2o_ext_csparse
set(G2O_LIBS g2o_types_slam3d g2o_solver_cholmod g2o_solver_pcg g2o_solver_csparse g2o_stuff g2o_core g2o_csparse_extension  g2o_solver_eigen g2o_solver_dense)
set(G2O_LIBRARIES ${G2O_LIBS} cholmod cxsparse)
# link_libraries(${G2O_LIBRARIES})
message("G2o-libraries "${G2O_LIBRARIES})

## Declare a cpp library
# add_library(g2o_exp
#   src/${PROJECT_NAME}/g2o_exp.cpp
# )

add_executable(test_slam3d_jacobian src/test_slam3d_jacobian.cpp)
target_link_libraries(test_slam3d_jacobian ${G2O_LIBRARIES} ${catkin_LIBRARIES})

add_executable(g2o_exp src/g2o_in_rgbdslam.cpp)
target_link_libraries(g2o_exp ${G2O_LIBRARIES} ${catkin_LIBRARIES})

add_executable(g2o_exp2 src/g2o_in_rgbdslam2.cpp)
target_link_libraries(g2o_exp2 ${G2O_LIBRARIES} ${catkin_LIBRARIES})

add_executable(test_slam3d_se3 src/test_slam3d_se3.cpp)
target_link_libraries(test_slam3d_se3 ${G2O_LIBRARIES} ${catkin_LIBRARIES})

## Declare a cpp executable
# add_executable(g2o_exp_node src/g2o_exp_node.cpp)



## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(g2o_exp_node g2o_exp_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(g2o_exp_node
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
# install(TARGETS g2o_exp g2o_exp_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_g2o_exp.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
