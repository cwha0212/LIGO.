cmake_minimum_required(VERSION 3.8)
project(ligo)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-std=c++17 -O3 -pthread -fexceptions)

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

# CPU / OpenMP
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
  include(ProcessorCount)
  ProcessorCount(N)
  if(N GREATER 5)
    add_compile_definitions(MP_EN MP_PROC_NUM=4)
  elseif(N GREATER 3)
    math(EXPR PROC_NUM "${N} - 2")
    add_compile_definitions(MP_EN MP_PROC_NUM=${PROC_NUM})
  else()
    add_compile_definitions(MP_PROC_NUM=1)
  endif()
else()
  add_compile_definitions(MP_PROC_NUM=1)
endif()
find_package(OpenMP QUIET)
if(OpenMP_CXX_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# ROS2 / ament
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LocalSensorExternalTrigger.msg"
  DEPENDENCIES std_msgs builtin_interfaces
)
ament_export_dependencies(rosidl_default_runtime)

# gnss_comm: optional. If not found, build without GNSS (LiDAR+IMU+NMEA only).
find_package(gnss_comm QUIET)
if(NOT gnss_comm_FOUND)
  message(STATUS "ligo: gnss_comm not found - building without GNSS support")
  add_compile_definitions(LIGO_WITHOUT_GNSS)
endif()

# Third-party
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(GTSAM REQUIRED QUIET)
find_package(Ceres REQUIRED)
find_package(Sophus REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${Python3_INCLUDE_DIRS}
)

# Source list (exclude GNSS/NMEA when building without gnss_comm)
set(ligo_sources
  src/laserMapping.cpp
  src/li_initialization.cpp
  src/parameters.cpp
  src/preprocess.cpp
  src/Estimator.cpp
  src/IMU_Processing.cpp
)
if(gnss_comm_FOUND)
  list(APPEND ligo_sources
    include/Urbannav_process/handler.cpp
    src/GNSS_Assignment.cpp
    src/GNSS_Initialization.cpp
    src/GNSS_Processing_fg.cpp
    src/NMEA_Assignment.cpp
    src/NMEA_Processing_fg.cpp
  )
  message(STATUS "ligo: building with gnss_comm")
else()
  add_compile_definitions(LIGO_WITHOUT_GNSS)
  list(APPEND ligo_sources src/ligo_gnss_nmea_stub.cpp)
  message(STATUS "ligo: building without gnss_comm (LiDAR+IMU only)")
endif()

add_executable(ligo_mapping ${ligo_sources})

ament_target_dependencies(ligo_mapping
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
  livox_ros_driver2
)
# Same-package msg: link generated interface target (ligo is not find_package'd while building)
target_link_libraries(ligo_mapping ${PROJECT_NAME}__rosidl_typesupport_cpp)

target_link_libraries(ligo_mapping
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  ${CERES_LIBRARIES}
  gtsam
  Sophus::Sophus
  fmt
)
if(gnss_comm_FOUND)
  target_link_libraries(ligo_mapping ${gnss_comm_LIBRARIES})
  target_include_directories(ligo_mapping PRIVATE ${gnss_comm_INCLUDE_DIRS})
endif()

target_include_directories(ligo_mapping PRIVATE
  ${Python3_INCLUDE_DIRS}
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

install(TARGETS ligo_mapping
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config rviz_cfg launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
