find_package(PCL 1.7 REQUIRED)
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
    ${PCL_INCLUDE_DIRS}
    ${Octomap_INCLUDE_DIRS}
)

list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(maav-save-octomap save-octomap.cpp)

add_executable(publish-octomap publish-saved-octomap.cpp)

target_link_libraries(maav-save-octomap
    maav-utils
    maav-msg
    VisionUtils
    ${ZCM_LIBRARIES}
    ${YAMLCPP_LIBRARY}
    ${Octomap_LIBRARIES}
)

target_link_libraries(publish-octomap
    maav-utils
    maav-msg
    VisionUtils
    ${ZCM_LIBRARIES}
    ${YAMLCPP_LIBRARY}
    ${Octomap_LIBRARIES}
)
