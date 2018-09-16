include_directories(
    ${ZCM_INCLUDE_DIRS}
)

add_library(maav-utils SHARED
    debug.cpp
    getopt.c
    GetOpt.cpp
    Log.cpp
    MsgValidator.cpp
    SerialTTY.cpp
    TimeSync.cpp
    Tracker.cpp
    zarray.c
    ZCMHandler.cpp
    zhash.c
)

target_link_libraries(maav-utils
    ${ZCM_LIBRARIES}
)

add_subdirectory(xbox-controller)
