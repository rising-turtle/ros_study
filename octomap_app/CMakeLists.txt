cmake_minimum_required(VERSION 2.8)
project(octomap_app)

#set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-fpermissive")

set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-g -w")

find_package(catkin REQUIRED COMPONENTS 
	nav_msgs roscpp rostest tf tf_conversions 
	image_transport camera_info_manager pcl_ros
 octomap_server_local
#octomap
       )

set(octomap_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../octomap_server_local/extern/octomap_feature/build/InstallFiles)
#set(octomap_DIR ${CMAKE_CURRENT_SOURCE_DIR}/extern/octomap_feature/build/InstallFiles)

find_package(octomap REQUIRED)
find_package(OpenCV REQUIRED)

set(SRCS
	src/rgbd2feat.cpp
	src/rgbd2ot.cpp
	src/imgIndex_kdTree_opencv.cpp
	src/TrajFilter.cpp
	)
# message("octomap dirs: ${OCTOMAP_INCLUDE_DIRS}")
# message("catkin_include_dirs: ${catkin_INCLUDE_DIRS}")
include_directories(
    ${catkin_INCLUDE_DIRS}
	${OCTOMAP_INCLUDE_DIRS}
	src
	include
)
catkin_package()

add_executable(main_flru_new src/main_flru_new.cpp ${SRCS})
target_link_libraries(main_flru_new  ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES})

add_executable(main_flru_feature src/main_flru_feature.cpp ${SRCS})
target_link_libraries(main_flru_feature  ${OpenCV_LIBRARIES} ${OCTOMAP_LIBRARIES} ${catkin_LIBRARIES})
