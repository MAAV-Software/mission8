include_directories(
    ${GNC_INCLUDE_DIR}
    ${GNC_SRC_DIR}
)

# Manual handheld D415 run
find_package(LibRealSense OPTIONAL)
if(EXISTS ${LIBREALSENSE_LIBRARIES})
    include_directories(
        ${LIBREALSENSE_INCLUDE_DIRS}
        ${OPENCV_INCLUDE_DIR}
    )

    add_executable(manual
        manual/main.cpp
    )

    target_link_libraries(manual
        ${LIBREALSENSE_LIBRARIES}
        ${OpenCV_LIBS}
        maav-gnc
    )
endif()

add_executable(tums
    tums/main.cpp
)

target_link_libraries(tums
    maav-gnc
    maav-slam
)

