if(NOT BUILD_PLOTTER)
    return()
endif()

find_package(Pangolin REQUIRED)

add_executable(maav-plotter plotter.cpp)

target_include_directories(maav-plotter PUBLIC
    ${Pangolin_INCLUDE_DIRS}
)

target_link_libraries(maav-plotter
    maav-msg
    ${ZCM_LIBRARIES}
    ${YAMLCPP_LIBRARY}
    ${Pangolin_LIBRARIES}
)