cmake_minimum_required(VERSION 2.8)
project(mapstitch)

find_package(catkin REQUIRED nav_msgs roscpp rostest tf)
find_package(OpenCV REQUIRED)

include_directories(
	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	src
	include
)

catkin_package()

add_executable(mapstitch src/main.cpp src/mapstitch.cpp)
target_link_libraries(mapstitch ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

add_executable(ros_mapstitch src/rosmain.cpp src/mapstitch.cpp)
target_link_libraries(ros_mapstitch ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
