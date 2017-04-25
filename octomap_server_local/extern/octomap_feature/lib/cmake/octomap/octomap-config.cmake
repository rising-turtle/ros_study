# ===================================================================================
#  The OctoMap CMake configuration file
#
#             ** File generated automatically, do not modify **
#
#  Usage from an external project:
#    In your CMakeLists.txt, add these lines:
#
#    FIND_PACKAGE(octomap REQUIRED )
#    TARGET_LINK_LIBRARIES(MY_TARGET_NAME ${OCTOMAP_LIBRARIES})
#
#    This file will define the following variables:
#      - OCTOMAP_LIBRARIES      : The list of libraries to links against.
#      - OCTOMAP_LIBRARY_DIRS   : The directory where lib files are. Calling
#                                 LINK_DIRECTORIES with this path is NOT needed.
#      - OCTOMAP_INCLUDE_DIRS   : The OpenCV include directories.
#
# Based on the example CMake Tutorial
# http://www.vtk.org/Wiki/CMake/Tutorials/How_to_create_a_ProjectConfig.cmake_file
# and OpenCVConfig.cmake.in from OpenCV
# ===================================================================================

 
set(OCTOMAP_INCLUDE_DIRS "/home/davidz/work/ros/hydro/src/octomap_server_local/extern/octomap_feature/include")
set(OCTOMAP_LIBRARY_DIRS "/home/davidz/work/ros/hydro/src/octomap_server_local/extern/octomap_feature/lib")
 

# Set library names as absolute paths:
set(OCTOMAP_LIBRARIES
  "/home/davidz/work/ros/hydro/src/octomap_server_local/extern/octomap_feature/lib/liboctomap.so"
  "/home/davidz/work/ros/hydro/src/octomap_server_local/extern/octomap_feature/lib/liboctomath.so"
)
