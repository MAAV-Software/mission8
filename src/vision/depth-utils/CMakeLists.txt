# find required packages
find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(LibRealSense2 REQUIRED)
find_package(Boost REQUIRED system)

list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

include_directories(
    ${OPENCV_INCLUDE_DIR}
    ${YAMLCPP_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
    ${LIBREALSENSE2_INCLUDE_DIRS}
)

include_directories(SYSTEM
    ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(data-log-check data-log-check.cpp DataLogReader.cpp)

add_executable(planeFitterPCLTest planeFitterPclTest.cpp)

add_executable(planeFitterIRLTest planeFitterTestIRL.cpp)

target_link_libraries(planeFitterIRLTest
    CameraInterface
    PlaneFitter
    maav-utils
    maav-msg
    ${YAMLCPP_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
    ${LIBREALSENSE2_LIBRARIES}
)

target_link_libraries(planeFitterPCLTest
    CameraInterface
    PlaneFitter
    maav-utils
    maav-msg
    ${YAMLCPP_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
    ${LIBREALSENSE2_LIBRARIES}
)

target_link_libraries(data-log-check
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    ${LIBREALSENSE2_LIBRARIES}
)
