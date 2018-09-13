add_library(maav-state SHARED
    base_state.cpp
)

target_include_directories(maav-state PUBLIC
    ${GNC_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${SOPHUS_INCLUDE_DIR}
)
