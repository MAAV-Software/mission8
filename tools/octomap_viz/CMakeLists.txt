if(NOT BUILD_VISUALIZER)
    return()
endif()

find_package(Pangolin REQUIRED)
find_package(OpenCV 3.0 QUIET)
find_package(Octomap REQUIRED)
find_package(PCL 1.7 REQUIRED)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

# Find the Qt library
set(Qt5_DIR "${PROJECT_SOURCE_DIR}/thirdparty/Qt5/5.12.1/gcc_64/lib/cmake/Qt5")
find_package(Qt5 5.12.1 REQUIRED COMPONENTS Core 
    PATHS ${SOFTWARE_SOURCE_DIR}/thirdparty
    NO_DEFAULT_PATH
    NO_CMAKE_FIND_ROOT_PATH
    QUIET)

add_executable(maav-octovis 
    Visualizer.cpp
)

target_include_directories(maav-octovis PUBLIC
    ${SW_INCLUDE_DIR}
	${OpenCV_INCLUDE_DIR}
    ${Pangolin_INCLUDE_DIRS}
    ${Octomap_INCLUDE_DIRS}
    ${Octovis_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

target_link_libraries(maav-octovis
    maav-msg
    maav-utils
    maav-state
    ${OpenCV_LIBS}
    ${ZCM_LIBRARIES}
    ${YAMLCPP_LIBRARY}
    ${Pangolin_LIBRARIES}
    ${Octomap_LIBRARIES}
    ${Octovis_LIBRARIES}
    Qt5::Core
    VisionUtils
)
