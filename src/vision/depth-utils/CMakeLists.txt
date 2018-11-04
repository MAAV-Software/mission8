# find required packages
find_package(PCL 1.7 REQUIRED)
find_package(GLM REQUIRED)
find_package(glfw3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(LibRealSense REQUIRED)
find_package(LibRealSense2 REQUIRED)

list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

include_directories(
    ${OPENCV_INCLUDE_DIR}
    ${ZCM_INCLUDE_DIR}
    ${YAMLCPP_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${LIBREALSENSE_INCLUDE_DIRS}
    ${LIBREALSENSE2_INCLUDE_DIRS}
    ${GLM_INCLUDE_DIRS}
    ${GLFW3_INCLUDE_DIR}
)

include_directories(SYSTEM
    ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(RGBDGetter SHARED RGBDGetter.cpp)

add_library(CameraInput SHARED CameraInputBase.cpp CameraInput.cpp NewCameraInput.cpp)

add_executable(data-log data-log.cpp)

add_library(PlaneFitterPCL SHARED PlaneFitterPCL.cpp)

add_library(PlaneFitter SHARED PlaneFitter)

target_link_libraries(CameraInput
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    ${LIBREALSENSE_LIBRARIES}
    ${LIBREALSENSE2_LIBRARIES}
)

target_link_libraries(CameraInput realsense2)

target_link_libraries(data-log
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    ${LIBREALSENSE_LIBRARIES}
    ${LIBREALSENSE2_LIBRARIES}
    realsense2
    CameraInput
)

target_link_libraries(PlaneFitterPCL
    ${PCL_LIBRARIES}
    ${LIBREALSENSE_LIBRARIES}
    ${LIBREALSENSE2_LIBRARIES}
)

target_link_libraries(RGBDGetter
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    ${LIBREALSENSE_LIBRARIES}
    ${LIBREALSENSE2_LIBRARIES}
)
