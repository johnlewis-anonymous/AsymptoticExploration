cmake_minimum_required(VERSION 2.8.3)
project(viewpoint_generator)

SET(CMAKE_BUILD_TYPE Release)
add_compile_options(-std=c++14)

FIND_PACKAGE(Boost REQUIRED
  date_time
  serialization
  thread
  filesystem
  system
  regex
  timer
  chrono
  program_options
  unit_test_framework
)



find_package(catkin REQUIRED COMPONENTS
  roscpp
  octomap_ros
  octomap_msgs
  pcl_ros
  pcl_conversions
  eigen_conversions
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  message_generation
  tf
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
  dynamic_reconfigure
)

find_package(OpenMP)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS} -fopenmp")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -fopenmp")



add_message_files(
  FILES
  Frontier.msg
  Viewpoint.msg
  FrontierArray.msg
  ViewpointArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    message_runtime
    pcl_ros
    pcl_conversions
    eigen_conversions
    diagnostic_msgs
    geometry_msgs
    nav_msgs
    sensor_msgs
    std_msgs
    tf
    tf2
    tf2_ros
    tf2_geometry_msgs
    tf2_sensor_msgs
    dynamic_reconfigure
    message_filters
    nodelet
    pluginlib
  DEPENDS
    Boost
)

include_directories(include 
  ${catkin_INCLUDE_DIRS}  
  ${Boost_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}  
  ${Boost_LIBRARY_DIRS}
)

set(VIEWPOINT_GENERATION_SRCS src/ViewpointGenerator.cc)

add_library(${PROJECT_NAME} ${VIEWPOINT_GENERATION_SRCS})
add_library(${PROJECT_NAME}_nodelet src/${PROJECT_NAME}_nodelet.cc)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_nodelet 
  ${PROJECT_NAME} 
  ${Boost_LIBRARIES} 
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cc)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)


# Ensure pluginlib knows about the nodelet
install(TARGETS ${PROJECT_NAME}_nodelet  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(FILES ${PROJECT_NAME}_plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})