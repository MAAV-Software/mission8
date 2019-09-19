# MAAV's include directory
include_directories(
    ${SW_INCLUDE_DIR}
    ${EIGEN_INCLUDE_DIRS}
)

add_subdirectory(common)
add_subdirectory(gcs-client)
add_subdirectory(gcs)
add_subdirectory(gnc)
add_subdirectory(lidar)
add_subdirectory(obstacles)
add_subdirectory(tanfan)
add_subdirectory(vision)
add_subdirectory(imu)
