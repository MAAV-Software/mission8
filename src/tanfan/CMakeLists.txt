if(NOT BUILD_TANFAN)
    return()
endif()

include_directories(
    ${ZCM_INCLUDE_DIR}
)

add_library(tanfan-nav-dep
    nav/PhysicalController.cpp
    nav/SerialTTY.cpp
    nav/Controller.cpp
    nav/data_link.c
    nav/Log.cpp
    nav/msg/gains_t.c
    nav/msg/setpt_t.c
    nav/msg/feedback_t.c
)

add_library(tanfan-messaging
    messaging/DataLink.cpp
    messaging/Decoder.cpp
    messaging/MessageHandler.cpp
    messaging/DataLinkDefines.cpp
    messaging/TransmitHandler.cpp
    messaging/RingBuffer.cpp
    messaging/imu_t.c
    messaging/lidar_t.c
    messaging/Encoder.cpp
    messaging/dji_t.c
    messaging/emergency_t.c
    lcmlite.cpp
)

