cmake_minimum_required(VERSION 3.0.2)
project(raycast_static_map)

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  std_srvs
  message_filters
  pcl_ros
  pcl_conversions
  tf2
  tf2_geometry_msgs
  visualization_msgs
)
find_package(PCL REQUIRED COMPONENTS common)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES raycast_mapper
  CATKIN_DEPENDS roscpp sensor_msgs nav_msgs std_srvs message_filters pcl_ros pcl_conversions tf2 tf2_geometry_msgs visualization_msgs
  DEPENDS PCL
)

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

add_library(raycast_mapper src/raycast_mapper.cpp src/object_tracker.cpp)
target_compile_features(raycast_mapper PUBLIC cxx_std_14)
target_link_libraries(raycast_mapper ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(raycast_static_map_node src/raycast_static_map_node.cpp)
target_compile_features(raycast_static_map_node PUBLIC cxx_std_14)
target_link_libraries(raycast_static_map_node raycast_mapper ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(doals_deskew_node src/doals_deskew_node.cpp)
target_compile_features(doals_deskew_node PUBLIC cxx_std_14)
target_link_libraries(doals_deskew_node ${catkin_LIBRARIES})

install(TARGETS raycast_mapper raycast_static_map_node doals_deskew_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY launch config rviz DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
catkin_install_python(PROGRAMS
  scripts/check_mapping_output.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(PROGRAMS scripts/download_doals_hauptgebaeude.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_raycast_mapper test/test_raycast_mapper.cpp)
  if(TARGET test_raycast_mapper)
    target_link_libraries(test_raycast_mapper raycast_mapper ${catkin_LIBRARIES})
  endif()
endif()
