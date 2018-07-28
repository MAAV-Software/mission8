include_directories(
    ${EIGEN3_INCLUDE_DIR}
    ${SOPHUS_INCLUDE_DIR}
)

add_library(maav-state SHARED
    base_state.cpp
)

