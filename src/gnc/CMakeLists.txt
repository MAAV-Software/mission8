# Exit if GNC should not be built
if(NOT BUILD_GNC)
    return()
endif()

# Set project-wide flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall -Werror -Wextra -pedantic")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")

# Find packages
find_package(Pangolin REQUIRED)
find_package(Sophus REQUIRED)
find_package(G2O REQUIRED)
find_package(DBOW2 REQUIRED)
find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

# Add source directories
add_subdirectory(state)
add_subdirectory(control)
add_subdirectory(kalman)
add_subdirectory(slam)
add_subdirectory(utils)

# build libmaav-gnc
add_library(maav-gnc SHARED
    localizer.cpp
    estimator.cpp
    planner.cpp
    controller.cpp
)

target_include_directories(maav-gnc PUBLIC
    ${SW_INCLUDE_DIR}
    ${G2O_INCLUDE_DIR}
    ${DBOW_INCLUDE_DIR}
    ${SOPHUS_INCLUDE_DIR}
    ${Eigen_INCLUDE_DIR}
	${OpenCV_INCLUDE_DIR}
)

target_link_libraries(maav-gnc
    maav-slam
    maav-state
    maav-kalman
	${OpenCV_LIBS}
    maav-gnc-utils
)
