find_package(PCL 1.7 REQUIRED)
find_package(LibRealSense REQUIRED)
find_package(LibRealSense2 REQUIRED)
find_package(ZLIB REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ZCM REQUIRED)
find_package(YAML REQUIRED)

list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

include_directories(
    ${OPENCV_INCLUDE_DIR}
    ${ZCM_INCLUDE_DIRS}
    ${YAMLCPP_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${LIBREALSENSE_INCLUDE_DIRS}
    ${LIBREALSENSE2_INCLUDE_DIRS}
)

add_library(VehicleStateRepo SHARED VehicleStateRepo.cpp)

target_link_libraries(VehicleStateRepo ${ZCM_LIBRARY})

add_executable(maav-vision
    maav-vision.cpp
    ImageFeed.cpp
    ZCMAccumulator.cpp
    VideoInterface.cpp
    DataCoordinator.cpp
    TimeInterface.cpp
    CameraIdentifier.cpp
)

target_link_libraries(maav-vision
    ${ZLIB_LIBRARIES}
    ${ZCM_LIBRARY}
    ${OpenCV_LIBS}
    ${YAMLCPP_LIBRARY}
    ${PCL_LIBRARIES}
    ${LIBREALSENSE_LIBRARIES}
    ${LIBREALSENSE2_LIBRARIES}
    nav-line-detector
    nav-roomba-detector
    atomcore-msg
    atomcore-common
    VehicleStateRepo
    RGBDGetter
    CameraInput
)
