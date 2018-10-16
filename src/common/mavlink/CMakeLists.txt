include_directories(maav-mavlink
    ${SW_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
)

add_library(maav-mavlink SHARED
    communication_port.cpp
    offboard_control.cpp
)
