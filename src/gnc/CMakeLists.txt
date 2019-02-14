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
    Controller.cpp
)

target_include_directories(maav-control PUBLIC 
    ${SW_INCLUDE_DIR}
)

target_link_libraries(maav-control
    maav-state
    maav-gnc-utils
    maav-measurements
    maav-msg
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

add_library(maav-slam SHARED
    slam/Converter.cc
    slam/Frame.cc
    slam/Initializer.cc
    slam/KeyFrame.cc
    slam/KeyFrameDatabase.cc
    slam/LocalMapping.cc
    slam/LoopClosing.cc
    slam/Map.cc
    slam/MapPoint.cc
    slam/Optimizer.cc
    slam/ORBextractor.cc
    slam/ORBmatcher.cc
    slam/PnPsolver.cc
    slam/Sim3Solver.cc
    slam/System.cc
    slam/Tracking.cc
    slam/VisualizerLink.cpp
    Localizer.cpp
)

target_include_directories(maav-slam PUBLIC
    ${SW_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${Pangolin_INCLUDE_DIRS}
    ${DBOW2_INCLUDE_DIR}
    ${G2O_INCLUDE_DIR}
	${OpenCV_INCLUDE_DIR}
    ${SOPHUS_INCLUDE_DIR}
)

target_link_libraries(maav-slam
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
    ${Pangolin_LIBRARIES}
    DBoW2
    g2o
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


# build libmaav-gnc
# add_library(maav-gnc SHARED
#     localizer.cpp
#     estimator.cpp
#     planner.cpp
#     controller.cpp
# )

# target_include_directories(maav-gnc PUBLIC
#     ${SW_INCLUDE_DIR}
#     ${G2O_INCLUDE_DIR}
#     ${DBOW_INCLUDE_DIR}
#     ${SOPHUS_INCLUDE_DIR}
#     ${Eigen_INCLUDE_DIR}
# 	${OpenCV_INCLUDE_DIR}
# )

# target_link_libraries(maav-gnc
#     maav-slam
#     maav-state
#     maav-kalman
# 	${OpenCV_LIBS}
#     maav-gnc-utils
#     maav-mavlink
#     maav-control
# )
