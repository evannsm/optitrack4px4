cmake_minimum_required(VERSION 3.8)
project(optitrack4px4)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(mocap_msgs REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)

###########################################################
# --- NatNet SDK vendored inside this package ---
set(NATNET_SDK_ROOT       "${CMAKE_CURRENT_SOURCE_DIR}/NatNetSDK")
set(NATNET_INCLUDE_DIR    "${NATNET_SDK_ROOT}/include")
set(NATNET_SDK_LIB        "${NATNET_SDK_ROOT}/lib/libNatNet.so")

if(NOT EXISTS "${NATNET_SDK_LIB}")
  message(FATAL_ERROR "NatNet SDK .so not found at: ${NATNET_SDK_LIB}")
endif()

# Imported target for NatNet SDK
add_library(natnet_sdk SHARED IMPORTED GLOBAL)
set_target_properties(natnet_sdk PROPERTIES
  IMPORTED_LOCATION "${NATNET_SDK_LIB}"
  INTERFACE_INCLUDE_DIRECTORIES "${NATNET_INCLUDE_DIR}"
)

###########################################################
# --- Utility library ---
add_library(optitrack4px4_utils
  src/utils.cpp
)
target_include_directories(optitrack4px4_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(optitrack4px4_utils PUBLIC
  Eigen3::Eigen
)

###########################################################
# --- optitrack_client executable ---
add_executable(optitrack_client
  src/communicator.cpp
  src/publisher.cpp
)

# Include our public headers when building this target
target_include_directories(optitrack_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link dependencies
target_link_libraries(optitrack_client
  optitrack4px4_utils
  natnet_sdk
  Threads::Threads
)

ament_target_dependencies(optitrack_client
  rclcpp
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  px4_msgs
  mocap_msgs
)

# Runtime search path so the binary finds libs in the install tree
set_target_properties(optitrack_client PROPERTIES
  INSTALL_RPATH "\$ORIGIN/../lib"
  BUILD_WITH_INSTALL_RPATH OFF
)

###########################################################
# --- Install rules ---
include(GNUInstallDirs)

# Install the NatNet SDK shared lib into install/lib
install(FILES "${NATNET_SDK_LIB}"
        DESTINATION ${CMAKE_INSTALL_LIBDIR})

# Install NatNet SDK headers
install(DIRECTORY "${NATNET_INCLUDE_DIR}/"
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
        FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")

# Install our public headers
install(DIRECTORY include/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# Install utility library
install(TARGETS optitrack4px4_utils
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

# Install all executables
install(TARGETS
  optitrack_client
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})

# Install config files
install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME})

ament_package()
