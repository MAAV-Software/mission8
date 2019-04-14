if(NOT BUILD_SIM)
    return()
endif()

find_package(gazebo REQUIRED)
find_package (Eigen3 REQUIRED)
find_package(ZCM REQUIRED)
find_package(Sophus REQUIRED)

include_directories(SYSTEM 
    ${GAZEBO_INCLUDE_DIRS} 
    ${ZCM_INCLUDE_DIRS} 
    ${EIGEN_INCLUDE_DIRS}
)

include_directories(${SW_INCLUDE_DIR})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

include_directories(
    ${PROJECT_SOURCE_DIR}/sim/plugins
    ${SOPHUS_INCLUDE_DIR}
)

set(PLUGIN_NAMES
    MaavImuPlugin
    MaavLidarPlugin
    MaavCameraPlugin
    MaavPlanefitPlugin
    MaavVehiclePlugin
    RealsenseT265Plugin
)


find_library(DepthCameraPlugin 
    NAMES DepthCameraPlugin
    PATHS ${GAZEBO_LIBRARY_DIRS}
)

# message(${GAZEBO_LIBRARY_DIRS})


foreach(PLUGIN_NAME ${PLUGIN_NAMES})
    set(PLUGIN_SRC "plugins/${PLUGIN_NAME}.cpp")
    add_library(${PLUGIN_NAME} SHARED ${PLUGIN_SRC})
    target_link_libraries(
        ${PLUGIN_NAME} 
        ${GAZEBO_LIBRARIES} 
        ${ZCM_LIBRARIES} 
        maav-msg
        maav-gnc-utils
        ${DepthCameraPlugin}
    )
endforeach(PLUGIN_NAME ${PLUGIN_NAMES})
