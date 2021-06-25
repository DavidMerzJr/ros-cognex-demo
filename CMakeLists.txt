cmake_minimum_required(VERSION 2.8.3)
project(cognex_demo)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  ros_opcua_impl_freeopcua
  roscpp
  sensor_msgs
  target_finder
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
  CATKIN_DEPENDS
    geometry_msgs
    ros_opcua_impl_freeopcua
    roscpp
    sensor_msgs
    target_finder
  DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(feature_detector_node src/feature_detector_node.cpp)
target_link_libraries(feature_detector_node ${catkin_LIBRARIES})

add_executable(location_solver_node src/location_solver_node.cpp)
target_link_libraries(location_solver_node ${catkin_LIBRARIES})

add_executable(plc_interface src/plc_interface.cpp)
target_link_libraries(plc_interface ${catkin_LIBRARIES})

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

foreach(dir config launch)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach()

install(TARGETS feature_detector_node location_solver_node plc_interface
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
