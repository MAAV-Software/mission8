# Exit if GNC should not be built
if(NOT BUILD_GNC)
    return()
endif()

# Find packages
find_package(Pangolin REQUIRED)
find_package(Sophus REQUIRED)
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

add_subdirectory(xbox-controller)

add_library(maav-measurements SHARED
    measurements/ImuMeasurement.cpp
    measurements/LidarMeasurement.cpp
    measurements/PlaneFitMeasurement.cpp
    measurements/GlobalUpdateMeasurement.cpp
)

include_directories(maav-gnc-utils
    ${SW_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${SOPHUS_INCLUDE_DIR}
)

add_library(maav-gnc-utils SHARED
    utils/ZcmConversion.cpp
    utils/LoadParameters.cpp
    utils/MagnetometerEllipsoidFit.cpp
)

target_link_libraries(maav-gnc-utils
    ${YAMLCPP_LIBRARY}
    maav-measurements
)

target_include_directories(maav-measurements PUBLIC
    ${SW_INCLUDE_DIR}
    ${SOPHUS_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(maav-measurements
    maav-state
)


add_library(maav-control SHARED
    control/Pid.cpp
    control/Controller.cpp
    control/StateMachine.cpp
)

target_include_directories(maav-control PUBLIC 
    ${SW_INCLUDE_DIR}
)

target_link_libraries(maav-control
    maav-state
    maav-gnc-utils
    maav-measurements
    maav-msg
    maav-mavlink
)

add_library(maav-kalman SHARED
    Estimator.cpp
    kalman/History.cpp
    kalman/Prediction.cpp
    kalman/Extrinsics.cpp
    kalman/updates/LidarUpdate.cpp
    kalman/updates/PlanefitUpdate.cpp
    kalman/updates/GlobalUpdate.cpp
)

target_include_directories(maav-kalman PUBLIC
    ${SW_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${SOPHUS_INCLUDE_DIR}
    ${YAMLCPP_INCLUDE_DIRS}
)

target_link_libraries(maav-kalman
    maav-state
    maav-measurements
    ${YAMLCPP_LIBRARY}
)

add_library(maav-guidance SHARED
    Planner.cpp
)

target_include_directories(maav-guidance PUBLIC
    ${SW_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${SOPHUS_INCLUDE_DIR}
)


target_include_directories(maav-kalman PUBLIC
    ${SW_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${SOPHUS_INCLUDE_DIR}
    ${YAMLCPP_INCLUDE_DIRS}
)

target_link_libraries(maav-kalman
    maav-state
    maav-measurements
    ${YAMLCPP_LIBRARY}
)
