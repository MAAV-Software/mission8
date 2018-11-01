# find required packages
find_package(OpenCV REQUIRED)
find_package(Apriltags REQUIRED)

include_directories(
    ${OPENCV_INCLUDE_DIR}
)

include_directories(SYSTEM
    ${APRILTAGS_INCLUDE_DIRS}
)

add_executable(opencv_demo opencv_demo.cc)

target_link_libraries(opencv_demo
    ${OpenCV_LIBS}
    ${APRILTAGS_LIBRARIES}
)
