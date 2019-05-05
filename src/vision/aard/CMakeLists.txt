# find required packages
find_package(OpenCV REQUIRED)
find_package(Apriltags REQUIRED)

add_executable(opencv_demo opencv_demo.cc)

target_link_libraries(opencv_demo
    ${OpenCV_LIBS}
    ${APRILTAGS_LIBRARIES}
)

target_include_directories(opencv_demo PUBLIC  ${OPENCV_INCLUDE_DIR})

target_include_directories(opencv_demo SYSTEM PUBLIC ${APRILTAGS_INCLUDE_DIRS})