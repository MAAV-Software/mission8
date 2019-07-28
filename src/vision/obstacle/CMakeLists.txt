# find required packages
find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ZCM REQUIRED)

list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

include_directories(
    ${OPENCV_INCLUDE_DIR}
    ${YAMLCPP_INCLUDE_DIRS}
    ${ZCM_INCLUDE_DIRS}
)

include_directories(SYSTEM
    ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(NaiveObstacleDetection SHARED
    NaiveObstacle.cpp
)

target_link_libraries(NaiveObstacleDetection
    ${OpenCV_LIBS}
    ${ZCM_LIBRARIES}
    ${PCL_LIBRARIES}
    VisionUtils
)
