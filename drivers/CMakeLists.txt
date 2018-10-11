include_directories(
        ${ZCM_INCLUDE_DIRS}
        ${SW_INCLUDE_DIR})

add_executable(maav-planner maav-guidance.cpp)

target_link_libraries(maav-planner maav-gnc
        ${ZCM_LIBRARIES}
        maav-utils
        maav-msg)

add_executable(maav-controller maav-controller.cpp)

target_link_libraries(maav-controller maav-gnc
        ${ZCM_LIBRARIES}
        maav-utils
        maav-msg)

add_executable(maav-estimator maav-estimator.cpp)

target_link_libraries(maav-estimator 
	maav-gnc
        ${ZCM_LIBRARIES}
        maav-utils
        maav-msg)


