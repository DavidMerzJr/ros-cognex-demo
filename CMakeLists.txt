cmake_minimum_required(VERSION 2.8.3)
project(cognex_demo)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  industrial_extrinsic_cal
  pcl_ros
  ros_opcua_impl_freeopcua
  roscpp
  sensor_msgs
  target_finder
  tf2
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
  CATKIN_DEPENDS
    geometry_msgs
    pcl_ros
    ros_opcua_impl_freeopcua
    roscpp
    sensor_msgs
    target_finder
    tf2
  DEPENDS
    OpenCV
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_library(plc_interface src/plc_interface.cpp)
target_link_libraries(plc_interface
  ${catkin_LIBRARIES}
)

add_executable(feature_detector_node src/feature_detector_node.cpp)
target_link_libraries(feature_detector_node
  ${catkin_LIBRARIES}
  plc_interface
)

add_executable(location_solver_node src/location_solver_node.cpp)
target_link_libraries(location_solver_node
  plc_interface
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(pointcloud_publisher src/pointcloud_publisher.cpp)
target_link_libraries(pointcloud_publisher
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

foreach(dir config launch support)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach()

install(TARGETS
    feature_detector_node
    location_solver_node
    plc_interface
    pointcloud_publisher
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
