if(NOT BUILD_OBSTACLES)
    return()
endif()


find_package(ZCM REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
    ${OPENCV_INCLUDE_DIR}
    ${EIGEN_INCLUDE_DIRS}
    ${ZCM_INCLUDE_DIR}
)

add_library(maav-obstacles SHARED
    ObstacleDetector.cpp
    lidar-viewer.cpp
)

add_executable(maav-obstacle
    maav-obstacle.cpp
)

target_link_libraries(maav-obstacle
    maav-msg
    maav-math
    maav-utils
    maav-lidar
    maav-obstacles
    ${OpenCV_LIBS}
    ${ZCM_LIBRARY}
)
