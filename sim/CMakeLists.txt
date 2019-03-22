if(NOT BUILD_SIM)
    return()
endif()

find_package(gazebo REQUIRED)
find_package (Eigen3 REQUIRED)
find_package(ZCM REQUIRED)

include_directories(SYSTEM 
    ${GAZEBO_INCLUDE_DIRS} 
    ${ZCM_INCLUDE_DIRS} 
    ${EIGEN_INCLUDE_DIRS}
)

include_directories(${SW_INCLUDE_DIR})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/sim/lib)

include_directories(
    ${PROJECT_SOURCE_DIR}/sim/sensors 
    ${PROJECT_SOURCE_DIR}/sim/plugins
)

set(PLUGIN_NAMES
    ground_truth_plugin
    MaavImuPlugin
    MaavLidarPlugin
    MaavCameraPlugin
    RealSensePlugin
    MaavPlaneFitPlugin
)

foreach(PLUGIN_NAME ${PLUGIN_NAMES})
    set(PLUGIN_SRC "plugins/${PLUGIN_NAME}.cpp")
    add_library(${PLUGIN_NAME} SHARED ${PLUGIN_SRC})
    target_link_libraries(
        ${PLUGIN_NAME} 
        ${GAZEBO_libraries} 
        ${ZCM_LIBRARIES} 
        maav-msg
    )
endforeach(PLUGIN_NAME ${PLUGIN_NAMES})


set(SENSOR_NAMES
    PlaneFitSensor
    SlamSensor)

foreach(SENSOR_NAME ${SENSOR_NAMES})
    set(SENSOR_SRC "sensors/${SENSOR_NAME}.cpp")
    add_library(${SENSOR_NAME} SHARED ${SENSOR_SRC})
    target_link_libraries(${SENSOR_NAME} ${GAZEBO_libraries} ${ZCM_LIBRARIES} maav-msg)
endforeach(SENSOR_NAME ${SENSOR_NAMES})

set(PLUGIN_NAME SensorRegisterPlugin)
set(PLUGIN_SRC "plugins/${PLUGIN_NAME}.cpp")
add_library(${PLUGIN_NAME} SHARED ${PLUGIN_SRC})
target_link_libraries(${PLUGIN_NAME} ${GAZEBO_libraries} ${ZCM_LIBRARIES} ${SENSOR_NAMES}  maav-msg)

