project(tests)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

include_directories(    
    ${GNC_INCLUDE_DIR}
    ${GNC_SRC_DIR}
)

# Manual handheld D415 run
set(LIBREALSENSE_INCLUDE_DIR /usr/local/include/librealsense2)
set(LIBREALSENSE_LIBRARY_DIR /usr/local/lib/librealsense2.so)
if(EXISTS ${LIBREALSENSE_LIBRARY_DIR})
    message("Found librealsense2")
    include_directories(${LIBREALSENSE_INCLUDE_DIR})
    
    add_executable(manual
        manual/main.cpp
    )

    target_link_libraries(manual
        ${LIBREALSENSE_LIBRARY_DIR}
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
   