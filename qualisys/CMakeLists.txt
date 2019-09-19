cmake_minimum_required(VERSION 2.8)
project(qualisys-zcm)

# Find eigen without using the Find script in software
find_package(Eigen3 QUIET)

# Set software source dir'
get_filename_component(SOFTWARE_SOURCE_DIR ${PROJECT_SOURCE_DIR} DIRECTORY)

# Add path to search for modules
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${SOFTWARE_SOURCE_DIR}/cmake")

# Set all compiled libraries to be placed in lib
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)

# Set all compiled executables to be placed in bin
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)

find_package(ZCM REQUIRED)
find_package(ZMQ REQUIRED)

# If Eigen was not found globally, try finding Eigen in software/thirdparty
if (NOT ${EIGEN_INCLUDE_DIRS})
    find_package(Eigen3 REQUIRED)
endif()

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAMLCPP REQUIRED yaml-cpp)

include_directories(
  include
  qualisys
  ${PROJECT_SOURCE_DIR}/../generated/common/messages
  ${YAMLCPP_INCLUDE_DIRS}
  ${ZCM_INCLUDE_DIRS}
  ${ZMQ_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-parameter -Wall -Wextra -pedantic -Werror")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

add_library(qualisys_driver SHARED
  src/State.cpp
  src/KalmanFilter.cpp
  src/QualisysEstimator.cpp
  src/QualisysZCM.cpp
  qualisys/RTProtocol.cpp
  qualisys/Markup.cpp
  qualisys/RTPacket.cpp
  qualisys/Network.cpp
)

target_link_libraries(qualisys_driver
  ${ZMQ_LIBRARY}
  ${YAMLCPP_LIBRARIES}
)
add_executable(maav-qualisys
  src/maav-qualisys.cpp
)

add_executable(maav-q2z-forwarder
  src/maav-q2z-forwarder.cpp
)

target_link_libraries(maav-qualisys
  qualisys_driver
)

target_link_libraries(maav-q2z-forwarder
  ${ZMQ_LIBRARY}
  ${ZCM_LIBRARY}
  ${YAMLCPP_LIBRARIES}
)
