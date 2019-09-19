find_package(OpenCV REQUIRED)

include_directories(
    ${EIGEN_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIR}
)

add_library(maav-lidar SHARED
    LaserReader.cpp
)

target_link_libraries(maav-lidar
    maav-math
    ${Boost_PROGRAM_OPTIONS_LIBRARY}
	${OpenCV_LIBS}
)
