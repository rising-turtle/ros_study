cmake_minimum_required(VERSION 2.8)
project(gmapping_app)

find_package(catkin REQUIRED nav_msgs openslam_gmapping roscpp rostest tf)

#find_package(Boost REQUIRED signals)

include_directories(${catkin_INCLUDE_DIRS})

include_directories(src)

include_directories(src/icp-mrpt)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/icp-mrpt)

catkin_package()

add_executable(main_gmapping src/main_gmapping.cpp)
target_link_libraries(main_gmapping ${catkin_LIBRARIES})

add_executable(main_gmapping_query src/main_gmapping_query.cpp)
target_link_libraries(main_gmapping_query ${catkin_LIBRARIES})

add_executable(main_gmapping_query_lo src/main_gmapping_query_lo.cpp src/icp-mrpt/point.cpp)
target_link_libraries(main_gmapping_query_lo ${catkin_LIBRARIES} zhicp)

add_executable(main_gmapping_flr src/main_gmapping_flr.cpp)
target_link_libraries(main_gmapping_flr ${catkin_LIBRARIES})

add_executable(main_gmapping_flrsyn src/main_gmapping_flrsyn.cpp)
target_link_libraries(main_gmapping_flrsyn ${catkin_LIBRARIES})

add_executable(main_fr079_query src/main_fr079_query.cpp)
target_link_libraries(main_fr079_query ${catkin_LIBRARIES})

add_executable(main_utaces_query src/main_utaces_query.cpp)
target_link_libraries(main_utaces_query ${catkin_LIBRARIES})

add_executable(main_stgate_query src/main_stgate_query.cpp)
target_link_libraries(main_stgate_query ${catkin_LIBRARIES})

add_executable(main_FLASER_query src/main_FLASER_query.cpp)
target_link_libraries(main_FLASER_query ${catkin_LIBRARIES})

