# find required packages
find_package(PCL 1.7 REQUIRED)
find_package(GLM REQUIRED)
find_package(glfw3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(LibRealSense REQUIRED)
find_package(LibRealSense2 REQUIRED)
find_package(Boost REQUIRED system)

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
    ${SW_INCLUDE_DIR}
    ${AUX_SW_INCLUDE_DIR}
)

include_directories(SYSTEM
    ${PCL_INCLUDE_DIRS}
    ${SOFTWARE_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(data-log-check data-log-check.cpp DataLogReader.cpp)

add_executable(snapshot getSnapshot.cpp)

add_executable(planeFitterPCLTest planeFitterPclTest.cpp)

add_executable(planeFitterIRLTest planeFitterTestIRL.cpp)

target_link_libraries(planeFitterIRLTest
    CameraInterface
    PlaneFitter
    maav-utils
    maav-msg
    ${YAMLCPP_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
    ${LIBREALSENSE_LIBRARIES}
    ${LIBREALSENSE2_LIBRARIES}
)

target_link_libraries(planeFitterPCLTest
    CameraInterface
    PlaneFitter
    maav-utils
    maav-msg
    ${YAMLCPP_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
    ${LIBREALSENSE_LIBRARIES}
    ${LIBREALSENSE2_LIBRARIES}
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

