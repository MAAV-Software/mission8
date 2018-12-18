# Exit if GNC should not be built
if(NOT BUILD_GNC)
    return()
endif()

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

add_library(maav-state SHARED
    State.cpp
)

target_include_directories(maav-state PUBLIC
    ${SW_INCLUDE_DIR}
    ${SOPHUS_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
)

# Add source directories
add_subdirectory(measurements)
add_subdirectory(control)
add_subdirectory(kalman)
add_subdirectory(slam)
add_subdirectory(utils)
add_subdirectory(xbox-controller)

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
    maav-mavlink
    maav-control
)
