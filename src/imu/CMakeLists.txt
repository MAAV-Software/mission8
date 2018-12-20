add_library(maav-imu-device SHARED
    Microstrain.cpp
)

target_include_directories(maav-imu-device PUBLIC
    ${LIBUSB_1_INCLUDE_DIRS}
)

target_link_libraries(maav-imu-device
    ${LIBUSB_1_LIBRARIES}
)