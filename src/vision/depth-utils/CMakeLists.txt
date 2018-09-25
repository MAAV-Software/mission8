#find required packages
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
	${PCL_INCLUDE_DIRS}
	${GLM_INCLUDE_DIRS}
    ${GLFW3_INCLUDE_DIR}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(RGBDGetter SHARED RGBDGetter.cpp)

add_library(CameraInput SHARED CameraInputBase.cpp CameraInput.cpp NewCameraInput.cpp)

add_executable(record-data record-data.cpp)

add_executable(RGBDGetterTest RGBDGetterTest.cpp RGBDGetter.cpp)

add_executable(CameraInputTest CameraInputTest.cpp)

add_executable(data-log data-log.cpp)

add_executable(data-log-check data-log-check.cpp DataLogReader.cpp)

add_executable(newCameraPractice newCameraPractice.cpp)

add_executable(snapshot getSnapshot.cpp)

add_library(PlaneFitterPCL SHARED PlaneFitterPCL.cpp)

add_library(PlaneFitter SHARED PlaneFitter)

add_executable(planeFitterTest planeFitterTest.cpp)

add_executable(planeFitterPCLTest planeFitterPclTest.cpp)

add_executable(planeFitterIRLTest planeFitterTestIRL.cpp)

target_link_libraries(CameraInput
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    ${LIBREALSENSE_LIBRARIES}
    ${LIBREALSENSE2_LIBRARIES}
)

target_link_libraries(planeFitterIRLTest
    PlaneFitterPCL
    CameraInput
)

target_link_libraries(planeFitterPCLTest
    PlaneFitter
    PlaneFitterPCL
)

target_link_libraries(CameraInput realsense2)

target_link_libraries(planeFitterTest PlaneFitter)

target_link_libraries(record-data
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    ${LIBREALSENSE_LIBRARIES}
    ${LIBREALSENSE2_LIBRARIES}
)

target_link_libraries(data-log
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    ${LIBREALSENSE_LIBRARIES}
    ${LIBREALSENSE2_LIBRARIES}
    realsense2
    CameraInput
)

target_link_libraries(data-log-check
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    ${LIBREALSENSE_LIBRARIES}
    ${LIBREALSENSE2_LIBRARIES}
)

target_link_libraries(snapshot
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    ${EPOXY_LIBRARIES}
    ${GLFW3_LIBRARIES}
    ${LIBREALSENSE_LIBRARIES}
    ${LIBREALSENSE2_LIBRARIES}
)

target_link_libraries(RGBDGetterTest
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    ${LIBREALSENSE_LIBRARIES}
    ${LIBREALSENSE2_LIBRARIES}
)

target_link_libraries(CameraInputTest
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

target_link_libraries(RGBDGetter
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    ${LIBREALSENSE_LIBRARIES}
    ${LIBREALSENSE2_LIBRARIES}
)

target_link_libraries(newCameraPractice
    ${OpenCV_LIBS}
    ${LIBREALSENSE2_LIBRARIES}
    realsense2
)

# RGBD Testing
add_executable(pcl_test pcl_test.cpp)
add_executable(view_cloud view_cloud.cpp)
add_executable(view_rgb rgbView.cpp)
add_executable(view_infrared depthView.cpp)
add_executable(combined_view combinedView.cpp)

target_link_libraries(pcl_test
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    ${LIBREALSENSE_LIBRARIES}
    ${LIBREALSENSE2_LIBRARIES}
)

target_link_libraries(view_cloud
    ${EPOXY_LIBRARIES}
    ${GLFW3_LIBRARIES}
    ${PCL_LIBRARIES}
    ${LIBREALSENSE_LIBRARIES}
    ${LIBREALSENSE2_LIBRARIES}
)

target_link_libraries(view_rgb
    ${LIBREALSENSE_LIBRARIES}
    ${LIBREALSENSE2_LIBRARIES}
    ${OpenCV_LIBS}
)

target_link_libraries(view_infrared
    ${LIBREALSENSE_LIBRARIES}
    ${LIBREALSENSE2_LIBRARIES}
    ${OpenCV_LIBS}
)

target_link_libraries(combined_view
    ${LIBREALSENSE_LIBRARIES}
    ${LIBREALSENSE2_LIBRARIES}
    ${OpenCV_LIBS}
)
