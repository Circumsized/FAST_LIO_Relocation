cmake_minimum_required(VERSION 3.8)
project(fast_lio)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_definitions(-DROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/")

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

message(STATUS "CPU architecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
  include(ProcessorCount)
  ProcessorCount(N)
  message(STATUS "Processor count: ${N}")
  if(N GREATER 4)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=3)
    message(STATUS "cores for MP: 3")
  elseif(N GREATER 3)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=2)
    message(STATUS "cores for MP: 2")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

find_package(OpenMP REQUIRED)
if(NOT OpenMP_CXX_FOUND)
  message(FATAL_ERROR "OpenMP (C++) is required by fast_lio but was not found.")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters registration)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Pose6D.msg"
  DEPENDENCIES builtin_interfaces
)

set(FAST_LIO_DEPENDENCIES
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  builtin_interfaces
  tf2
  tf2_ros
  tf2_eigen
  pcl_conversions
  livox_ros_driver2
)

set(FAST_LIO_SOURCES
  src/preprocess.cpp
  src/ndt_relocalizer.cpp
  include/ikd-Tree/ikd_Tree.cpp
)

add_library(fast_lio_core SHARED
  ${FAST_LIO_SOURCES}
)
target_include_directories(fast_lio_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(fast_lio_core ${FAST_LIO_DEPENDENCIES})
rosidl_get_typesupport_target(cpp_typesupport_c ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(fast_lio_core
  PRIVATE "${cpp_typesupport_c}"
  PUBLIC ${PCL_LIBRARIES} Eigen3::Eigen OpenMP::OpenMP_CXX
)

add_executable(fastlio_mapping src/fastlio_node.cpp)
target_include_directories(fastlio_mapping PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(fastlio_mapping ${FAST_LIO_DEPENDENCIES})
target_link_libraries(fastlio_mapping fast_lio_core "${cpp_typesupport_c}" ${PCL_LIBRARIES})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_state_machine test/test_state_machine.cpp)
  target_include_directories(test_state_machine PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src/localization
    ${EIGEN3_INCLUDE_DIR}
  )
  target_link_libraries(test_state_machine fast_lio_core Eigen3::Eigen)
endif()

install(TARGETS fastlio_mapping fast_lio_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.hpp"
)
install(DIRECTORY launch config rviz_cfg DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

ament_export_dependencies(${FAST_LIO_DEPENDENCIES} rosidl_default_runtime)
ament_package()
