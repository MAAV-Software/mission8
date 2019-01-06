include_directories(maav-mavlink
    ${SW_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${ZCM_INCLUDE_DIRS}
)

add_library(maav-mavlink SHARED
    communication_port.cpp
    offboard_control.cpp
)

target_link_libraries(maav-mavlink
    maav-msg
    ${ZCM_LIBRARIES}
)