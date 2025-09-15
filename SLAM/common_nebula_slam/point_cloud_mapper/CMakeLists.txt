#check if z is getting updated
#print stuff to see the changed x,y, and z
cmake_minimum_required(VERSION 2.8.3)
project(point_cloud_mapper)

SET(CMAKE_BUILD_TYPE Release)

find_package(PCL REQUIRED COMPONENTS common octree filters)
if(NOT PCL_FOUND)
  message(FATAL_ERROR "This program requires the PCL library.")
endif(NOT PCL_FOUND)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  std_msgs
  core_msgs
  parameter_utils
  geometry_utils
  frontend_utils
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    tf
    std_msgs
    parameter_utils
    geometry_utils
    frontend_utils
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
  ${PCL_LIBRARY_DIRS}
)

add_library(${PROJECT_NAME} src/PointCloudMapper.cc
                            src/PointCloudMultiThreadedMapper.cc
                            src/SimplePointCloudMapper.cc
                            src/PointCloudIkdTreeMapper.cc
                            src/ikd_tree/ikd_Tree.cpp
                            )
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  OpenMP::OpenMP_CXX
)

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}.cc)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

# Add Gtest for the package
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_${PROJECT_NAME} test/test_${PROJECT_NAME}.cpp)
  target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME} ${catkin_LIBRARIES})  
endif(CATKIN_ENABLE_TESTING)