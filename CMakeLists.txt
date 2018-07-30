cmake_minimum_required (VERSION 3.10)
project(maav-gnc)

# Set a default build type if none was specified
set(RELEASE Release)
set(DEBUG Debug)
set(DEFAULT_BUILD_TYPE ${RELEASE})

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to '${DEFAULT_BUILD_TYPE}' as none was specified.")
    set(CMAKE_BUILD_TYPE ${DEFAULT_BUILD_TYPE})
endif()


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall -Werror")
if(${CMAKE_BUILD_TYPE} MATCHES ${RELEASE})
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
endif()

# Find packages
find_package (Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

# Build local packages
add_subdirectory(Thirdparty)
set(G2O_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/Thirdparty/g2o)
set(DBOW_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2)
set(SOPHUS_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/Thirdparty/Sophus)

set(GNC_SRC_DIR ${PROJECT_SOURCE_DIR}/src)

# Build submodules
add_subdirectory(src)

set(GNC_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)
set(GNC_LIB_DIR ${PROJECT_SOURCE_DIR}/lib)
set(GNC_TEST_DIR ${PROJECT_SOURCE_DIR}/tests)

# Build libmaav-gnc
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${GNC_LIB_DIR})

include_directories(
    ${PROJECT_SOURCE_DIR}
    ${GNC_SRC_DIR}
    ${GNC_INCLUDE_DIR}
    ${G2O_INCLUDE_DIR}
    ${DBOW_INCLUDE_DIR}
    ${SOPHUS_INCLUDE_DIR}
)

add_library(maav-gnc SHARED
    ${GNC_SRC_DIR}/gnc.cpp
)

target_link_libraries(maav-gnc 
    maav-slam
    maav-state
    maav-kalman
)


# Build tests and examples
enable_testing() 
add_subdirectory(tests)
add_subdirectory(examples)

# TODO: expose shared library and public headers

# set_target_properties(maav-gnc PROPERTIES PUBLIC_HEADER
#                       include/gnc.hpp)
# target_include_directories(maav-gnc PRIVATE src)
# target_include_directories(maav-gnc PRIVATE include)

