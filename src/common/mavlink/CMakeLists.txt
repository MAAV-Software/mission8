include_directories(maav-mavlink
    ${SW_INCLUDE_DIR}
    ${EIGEN_INCLUDE_DIRS}
    ${ZCM_INCLUDE_DIRS}
)

add_library(maav-mavlink SHARED
    SerialPort.cpp
    AutopilotInterface.cpp
)

target_link_libraries(maav-mavlink
    maav-msg
    maav-utils
    maav-math
    ${ZCM_LIBRARIES}
)