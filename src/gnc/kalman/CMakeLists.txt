add_library(maav-kalman SHARED
    kalman_state.cpp
    history.cpp
    unscented_transform.cpp
    updates/lidar_update.cpp
)

target_include_directories(maav-kalman PUBLIC
    ${SW_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${SOPHUS_INCLUDE_DIR}
)

target_link_libraries(maav-kalman
    maav-state
)
