find_package(OpenCV REQUIRED)
find_package(Octomap REQUIRED)

if(NOT BUILD_GNC)
    return()
endif()

include_directories(
    ${ZCM_INCLUDE_DIRS}
    ${SW_INCLUDE_DIR}
    ${OPENCV_INCLUDE_DIRS}
)

include_directories(
    SYSTEM
    ${Octomap_INCLUDE_DIRS}
)

add_executable(tool-octomap-creation createOctomap.cpp)

add_executable(tool-planner-visualization visualizePlanner.cpp)

target_link_libraries(tool-octomap-creation
    maav-utils
    maav-msg
    VisionUtils
    ${ZCM_LIBRARIES}
    ${YAMLCPP_LIBRARY}
    ${Octomap_LIBRARIES}
)

target_link_libraries(tool-planner-visualization
    maav-utils
    maav-msg
    VisionUtils
    ${ZCM_LIBRARIES}
    ${YAMLCPP_LIBRARY}
    ${Octomap_LIBRARIES}
)
