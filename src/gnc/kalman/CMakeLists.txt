add_library(maav-kalman SHARED
    history.cpp
    unscented_transform.cpp
    updates/lidar_update.cpp
    prediction.cpp
)

target_include_directories(maav-kalman PUBLIC
    ${SW_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${SOPHUS_INCLUDE_DIR}
    ${YAMLCPP_INCLUDE_DIRS}
)

target_link_libraries(maav-kalman
    maav-state
    maav-measurements
    ${YAMLCPP_LIBRARY}

)
