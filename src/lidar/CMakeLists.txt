include_directories(
    ${EIGEN3_INCLUDE_DIR}
)

add_library(maav-lidar SHARED
    LaserReader.cpp
)

target_link_libraries(maav-lidar
    maav-math
    ${Boost_PROGRAM_OPTIONS_LIBRARY}
)
