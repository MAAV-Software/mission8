include_directories(
        ${ZCM_INCLUDE_DIRS}
        ${SW_INCLUDE_DIR})

add_executable(maav-planner maav-guidance.cpp)

target_link_libraries(maav-planner maav-gnc
        ${ZCM_LIBRARIES}
        maav-utils
        maav-msg)