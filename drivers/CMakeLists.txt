find_package(LibRealSense)
find_package(LibRealSense2)

include_directories(
        ${ZCM_INCLUDE_DIRS}
        ${SW_INCLUDE_DIR}
        ${YAMLCPP_INCLUDE_DIRS}
)

add_executable(maav-planner maav-guidance.cpp)

target_link_libraries(maav-planner 
        maav-gnc
        ${ZCM_LIBRARIES}
        maav-utils
        maav-msg)

add_executable(maav-controller maav-controller.cpp)

target_link_libraries(maav-controller 
        maav-gnc
        ${ZCM_LIBRARIES}
        ${YAMLCPP_LIBRARY}
        maav-utils
        maav-msg
        maav-mavlink
        maav-gamepad
)

add_executable(maav-estimator maav-estimator.cpp)

target_link_libraries(maav-estimator 
	maav-gnc
        ${ZCM_LIBRARIES}
        maav-utils
        maav-msg
)

add_executable(maav-localizer maav-localizer.cpp)

target_link_libraries(maav-localizer
        maav-gnc
        ${ZCM_LIBRARIES}
        maav-utils
        maav-msg
)

if(EXISTS ${LIBREALSENSE_LIBRARIES})
    include_directories(
        ${LIBREALSENSE_INCLUDE_DIRS}
        ${LIBREALSENSE2_INCLUDE_DIRS}
        ${OPENCV_INCLUDE_DIR}
    )

    add_executable(manual
        main.cpp
    )

    target_link_libraries(manual
        ${LIBREALSENSE_LIBRARIES}
        ${LIBREALSENSE2_LIBRARIES}
        ${OpenCV_LIBS}
        ${Pangolin_LIBRARIES}
        ${ZCM_LIBRARIES}
        maav-msg
    )
endif()