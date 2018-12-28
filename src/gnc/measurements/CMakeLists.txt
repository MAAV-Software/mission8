add_library(maav-measurements SHARED
    Lidar.cpp
    GlobalUpdate.cpp
)

target_include_directories(maav-measurements PUBLIC
    ${SW_INCLUDE_DIR}
    ${SOPHUS_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(maav-measurements
    maav-state
)
