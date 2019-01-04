if(NOT BUILD_VISUALIZER)
    return()
endif()

find_package(Pangolin REQUIRED)
find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

add_executable(maav-visualizer 
    Visualizer.cpp
    Viewer.cpp
    FrameDrawer.cpp
    MapDrawer.cpp
)

target_include_directories(maav-visualizer PUBLIC
    ${SW_INCLUDE_DIR}
	${OpenCV_INCLUDE_DIR}
    ${Pangolin_INCLUDE_DIRS}
)

target_link_libraries(maav-visualizer
    maav-msg
    maav-utils
    maav-state
    maav-gnc-utils
    maav-slam
    ${OpenCV_LIBS}
    ${ZCM_LIBRARIES}
    ${YAMLCPP_LIBRARY}
    ${Pangolin_LIBRARIES}
)
