set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fvisibility=hidden")

find_package(udev REQUIRED)

include_directories(
    ${SW_INCLUDE_DIR}
    ${UDEV_INCLUDE_DIR}
)

add_library(maav-gamepad SHARED
    gamepad.c
)

target_link_libraries(maav-gamepad
    ${UDEV_LIBRARY}
)
