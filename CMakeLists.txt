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
find_package(ament_index_cpp REQUIRED)
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

# gnss_comm: optional. Required for NMEA factor pipeline; if not found, LiDAR+IMU stub only.
find_package(gnss_comm QUIET)

# Third-party
find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(OpenCV REQUIRED)
# Minimum: Ceres 2.2.0 (ceres/manifold.h, EigenQuaternionManifold), GTSAM 4.1.1.
# Override search: -DCeres_DIR=... -DGTSAM_DIR=... or CMAKE_PREFIX_PATH to install roots.
find_package(Ceres 2.2.0 REQUIRED)
if(CERES_VERSION VERSION_LESS 2.2.0)
  message(FATAL_ERROR
    "ligo requires Ceres Solver >= 2.2.0 (reported '${CERES_VERSION}'). "
    "Build/install Ceres 2.2.0+ and point Ceres_DIR to its build or install tree.")
endif()
find_package(GTSAM 4.1.1 REQUIRED CONFIG)
find_package(Sophus REQUIRED)
message(STATUS "ligo: Ceres ${CERES_VERSION}")
if(DEFINED GTSAM_VERSION_STRING)
  message(STATUS "ligo: GTSAM ${GTSAM_VERSION_STRING}")
endif()

# small_gicp: prefer building bundled thirdparty/small_gicp (koide3/small_gicp) with CMake,
# then optional include-only / system installs.
set(LIGO_THIRDPARTY_SMALL_GICP_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/small_gicp")
set(SMALL_GICP_INCLUDE_DIR "" CACHE PATH "Path to small_gicp include directory (folder containing small_gicp/...)")
set(LIGO_WITH_SMALL_GICP OFF)
set(LIGO_SMALL_GICP_MODE "disabled")
set(LIGO_SMALL_GICP_TARGET "")

if(EXISTS "${LIGO_THIRDPARTY_SMALL_GICP_ROOT}/CMakeLists.txt")
  # Static helper lib avoids shipping a separate small_gicp shared object with colcon install.
  set(_ligo_saved_build_shared_libs "${BUILD_SHARED_LIBS}")
  set(BUILD_SHARED_LIBS OFF CACHE BOOL "small_gicp: static lib linked into ligo_mapping" FORCE)
  set(BUILD_HELPER ON CACHE BOOL "small_gicp: build registration helper" FORCE)
  set(BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(BUILD_BENCHMARKS OFF CACHE BOOL "" FORCE)
  set(BUILD_PYTHON_BINDINGS OFF CACHE BOOL "" FORCE)
  add_subdirectory(
    "${LIGO_THIRDPARTY_SMALL_GICP_ROOT}"
    "${CMAKE_CURRENT_BINARY_DIR}/small_gicp_build"
  )
  if(DEFINED _ligo_saved_build_shared_libs)
    set(BUILD_SHARED_LIBS "${_ligo_saved_build_shared_libs}" CACHE BOOL "" FORCE)
  endif()
  unset(_ligo_saved_build_shared_libs)
  set(LIGO_WITH_SMALL_GICP ON)
  set(LIGO_SMALL_GICP_MODE "bundled-subbuild")
  if(TARGET small_gicp)
    set(LIGO_SMALL_GICP_TARGET "small_gicp")
  endif()
elseif(EXISTS "${LIGO_THIRDPARTY_SMALL_GICP_ROOT}/include/small_gicp/registration/registration.hpp")
  set(LIGO_WITH_SMALL_GICP ON)
  set(SMALL_GICP_INCLUDE_DIR "${LIGO_THIRDPARTY_SMALL_GICP_ROOT}/include")
  set(LIGO_SMALL_GICP_MODE "bundled-headers")
elseif(SMALL_GICP_INCLUDE_DIR AND EXISTS "${SMALL_GICP_INCLUDE_DIR}/small_gicp/registration/registration.hpp")
  set(LIGO_WITH_SMALL_GICP ON)
  set(LIGO_SMALL_GICP_MODE "external-include-path")
else()
  find_package(small_gicp QUIET)
  if(small_gicp_FOUND)
    set(LIGO_WITH_SMALL_GICP ON)
    set(LIGO_SMALL_GICP_MODE "find-package")
    if(TARGET small_gicp::small_gicp)
      set(LIGO_SMALL_GICP_TARGET "small_gicp::small_gicp")
    endif()
  endif()
endif()

if(LIGO_WITH_SMALL_GICP)
  message(STATUS "ligo: small_gicp enabled (${LIGO_SMALL_GICP_MODE})")
else()
  message(STATUS "ligo: small_gicp not found (indoor small_gicp localizer module disabled)")
endif()

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)
if(SMALL_GICP_INCLUDE_DIR AND NOT LIGO_SMALL_GICP_TARGET)
  include_directories(${SMALL_GICP_INCLUDE_DIR})
endif()

# Source list: NMEA (PPP) when gnss_comm is available; otherwise stub GNSSProcess/NMEAProcess.
set(ligo_sources
  src/laserMapping.cpp
  src/li_initialization.cpp
  src/parameters.cpp
  src/Indoor_NoiseModel.cpp
  src/Indoor_Processing.cpp
  src/preprocess.cpp
  src/Estimator.cpp
  src/IMU_Processing.cpp
)
if(gnss_comm_FOUND)
  add_compile_definitions(LIGO_WITH_NMEA)
  list(APPEND ligo_sources
    src/NMEA_Assignment.cpp
    src/NMEA_Processing_fg.cpp
  )
  message(STATUS "ligo: building with NMEA (gnss_comm)")
else()
  list(APPEND ligo_sources src/ligo_gnss_nmea_stub.cpp)
  message(STATUS "ligo: gnss_comm not found - LiDAR+IMU only (NMEA stub)")
endif()
if(LIGO_WITH_SMALL_GICP)
  list(APPEND ligo_sources
    src/Indoor_SmallGICP_Localizer.cpp
    src/Indoor_GridMapRegistry.cpp
  )
  add_compile_definitions(LIGO_WITH_SMALL_GICP)
endif()

add_executable(ligo_mapping ${ligo_sources})

# GTSAM 4.1 API: OptionalMatrixType / default Jacobian args (see ligo_gtsam_compat.h)
target_compile_options(ligo_mapping PRIVATE
  "-include${CMAKE_CURRENT_SOURCE_DIR}/include/ligo_gtsam_compat.h")

ament_target_dependencies(ligo_mapping
  ament_index_cpp
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
if(LIGO_SMALL_GICP_TARGET)
  target_link_libraries(ligo_mapping ${LIGO_SMALL_GICP_TARGET})
endif()
if(gnss_comm_FOUND)
  target_link_libraries(ligo_mapping ${gnss_comm_LIBRARIES})
  target_include_directories(ligo_mapping PRIVATE ${gnss_comm_INCLUDE_DIRS})
endif()

target_include_directories(ligo_mapping PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

install(TARGETS ligo_mapping
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/grid_membership_monitor.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config rviz_cfg launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY docs
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY PCD
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
