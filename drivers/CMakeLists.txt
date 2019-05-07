find_package(LibRealSense2)
find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(LibRealSense2 REQUIRED)
find_package(Eigen3 REQUIRED)

list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

include_directories(
        ${ZCM_INCLUDE_DIRS}
        ${SW_INCLUDE_DIR}
        ${OPENCV_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
)

include_directories(
    SYSTEM
    ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

if(BUILD_GNC)

add_executable(maav-planner maav-guidance.cpp)

target_link_libraries(maav-planner
        maav-guidance
        maav-gnc-utils
        maav-measurements
        ${YAMLCPP_LIBRARY}
        ${ZCM_LIBRARIES}
        maav-utils
        maav-msg
)

add_executable(maav-controller maav-controller.cpp)

target_link_libraries(maav-controller
        maav-control
        maav-gnc-utils
        maav-measurements
        ${ZCM_LIBRARIES}
        ${YAMLCPP_LIBRARY}
        maav-utils
        maav-msg
        maav-mavlink
        maav-gamepad
        maav-math
)

add_executable(maav-estimator maav-estimator.cpp)

target_link_libraries(maav-estimator
        maav-kalman
        maav-gnc-utils
        ${ZCM_LIBRARIES}
        maav-utils
        maav-msg
)

add_executable(maav-control-input maav-control-input.cpp)

target_link_libraries(maav-control-input
        maav-control
        maav-gnc-utils
        maav-measurements
        ${ZCM_LIBRARIES}
        maav-utils
        maav-msg
)

endif()

link_directories(${PCL_LIBRARY_DIRS})

add_executable(maav-planefit maav-planefit.cpp)

target_link_libraries(maav-planefit
        ${ZCM_LIBRARIES}
        maav-utils
        maav-msg
        ${PCL_LIBRARIES}
        ${OpenCV_LIBS}
        ${LIBREALSENSE2_LIBRARIES}
        PlaneFitter
)


if(BUILD_TANFAN)

add_executable(tanfan tanfan.cpp)

target_link_libraries(tanfan
        ${ZCM_LIBRARIES}
        maav-utils
        tanfan-messaging
        tanfan-nav-dep
        maav-msg
)

#add_executable(spy-tanfan spy-tanfan.cpp)
#
#target_link_libraries(spy-tanfan
#        ${ZCM_LIBRARIES}
#        maav-utils
#        maav-msg
#)

endif()

if(BUILD_IMU)

add_executable(maav-imu maav-imu.cpp)

target_link_libraries(maav-imu
        ${ZCM_LIBRARIES}
        maav-msg
        maav-utils
        maav-gnc-utils
        ${YAMLCPP_LIBRARY}
        maav-imu-device
)

endif()



add_executable(maav-killswitch maav-killswitch.cpp)

target_link_libraries(maav-killswitch
        ${ZCM_LIBRARIES}
        maav-msg
        maav-utils
)

add_executable(read-thrust read-thrust.cpp)

target_link_libraries(read-thrust
        ${ZCM_LIBRARIES}
        ${YAMLCPP_LIBRARY}
        maav-msg
        maav-utils
)

if(BUILD_VISION)

    add_executable(maav-camera maav-camera.cpp)

    target_link_libraries(maav-camera
        ${ZCM_LIBRARIES}
        maav-utils
        maav-msg
        ${PCL_LIBRARIES}
        ${OpenCV_LIBS}
        ${LIBREALSENSE2_LIBRARIES}
        ${YAMLCPP_LIBRARY}
        CameraDriverHelper
    )

endif()
