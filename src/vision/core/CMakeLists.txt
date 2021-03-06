# find required packages
find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(LibRealSense2 REQUIRED)
find_package(ZCM REQUIRED)
find_package(libusb-1.0 REQUIRED)
find_package(Octomap REQUIRED)

list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

include_directories(
    ${OPENCV_INCLUDE_DIR}
    ${YAMLCPP_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
    ${LIBREALSENSE2_INCLUDE_DIRS}
    ${ZCM_INCLUDE+DIRS}
)

include_directories(SYSTEM
    ${PCL_INCLUDE_DIRS}
    ${Octomap_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(RealsenseSettings SHARED RealsenseSettings.cpp)

# add_library(RGBDPlayback SHARED RGBDPlayback.cpp)

add_library(CameraInterface SHARED
    CameraInterfaceBase.cpp
    D400CameraInterface.cpp
)

add_library(CameraDriverHelper SHARED CameraDriverHelper.cpp)

add_library(VisionUtils SHARED utilities.cpp)

# add_executable(data-log data-log.cpp)

add_library(PlaneFitter SHARED PlaneFitter.cpp)

target_link_libraries(CameraInterface
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    ${LIBREALSENSE2_LIBRARIES}
    ${LIBUSB_1_LIBRARIES}
    RealsenseSettings
    m
)

target_link_libraries(CameraInterface realsense2)

target_link_libraries(CameraDriverHelper
    CameraInterface
    VisionUtils
    maav-gnc-utils
)

target_link_libraries(PlaneFitter
    ${PCL_LIBRARIES}
    ${LIBREALSENSE2_LIBRARIES}
)

target_link_libraries(VisionUtils
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    ${Octomap_LIBRARIES}
)
