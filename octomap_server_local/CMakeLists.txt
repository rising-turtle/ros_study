cmake_minimum_required(VERSION 2.8)
project(octomap_server_local)

set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-g")

find_package(catkin REQUIRED pcl_ros nav_msgs roscpp tf)

set(octomap_DIR ${CMAKE_CURRENT_SOURCE_DIR}/extern/octomap_feature/build/InstallFiles)
find_package(octomap REQUIRED)

# message("octomap dirs: ${OCTOMAP_INCLUDE_DIRS}")
include_directories(
	${catkin_INCLUDE_DIRS}
	${OCTOMAP_INCLUDE_DIRS}
	src
	include
)
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS pcl_ros nav_msgs roscpp tf
   DEPENDS eigen #octomap 
)


add_library(octomap_server_local src/ColorOctreeImpl.cpp src/octo_globaldef.cpp
	src/colortable.cpp src/main.cpp)
target_link_libraries(octomap_server_local ${OCTOMAP_LIBRARIES} ${catkin_LIBRARIES})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
add_executable(main src/main.cpp )
target_link_libraries(main octomap_server_local ${OCTOMAP_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


## Mark executables and/or libraries for installation
install(TARGETS octomap_server_local 
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)


