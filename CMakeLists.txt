cmake_minimum_required(VERSION 2.8.3)
project(force_torque_sensor)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  dynamic_reconfigure
  filters
  geometry_msgs
  hardware_interface
  iirob_filters
  pluginlib
  realtime_tools
  roscpp
  rosparam_handler
  rospy
  std_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
else()
  set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
  set(Eigen_LIBRARIES ${EIGEN3_LIBRARIES})
endif()

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" Cpp11CompilerFlag)
if (${Cpp11CompilerFlag})
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()

generate_ros_parameter_files(
    cfg/CoordinateSystemCalibration.params
    cfg/CanConfiguration.params
    cfg/FTSConfiguration.params
    cfg/NodeConfiguration.params
    cfg/PublishConfiguration.params
    cfg/Calibration.params
)

add_service_files(
  FILES
  CalculateAverageMasurement.srv
  CalculateSensorOffset.srv
  DiagnosticVoltages.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS geometry_msgs iirob_filters message_runtime roscpp std_msgs std_srvs rospy tf2 tf2_ros tf2_geometry_msgs realtime_tools filters pluginlib
 DEPENDS Eigen
 LIBRARIES ${PROJECT_NAME} ${PROJECT_NAME}_sim
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/force_torque_sensor_handle.cpp )
add_dependencies(${PROJECT_NAME}
  ${PROJECT_NAME}_gencfg  # For dynamic_reconfigure
  ${PROJECT_NAME}_generate_messages_cpp
  ${PROJECT_NAME}_genparam # For rosparam_handler
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

add_library(${PROJECT_NAME}_sim src/force_torque_sensor_sim.cpp )
target_link_libraries(${PROJECT_NAME}_sim ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

add_executable(${PROJECT_NAME}_node src/force_torque_sensor_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

#############
## Install ##
#############
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_sim
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config/ cfg/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES force_torque_sensor_plugin.xml
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

