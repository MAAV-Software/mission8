include_directories(
    ${GNC_SRC_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${SOPHUS_INCLUDE_DIR}
)

add_library(maav-kalman SHARED
    kalman_state.cpp
)

target_link_libraries(
    maav-state
)
