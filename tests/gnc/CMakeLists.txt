project(tests)
enable_testing()
ADD_DEFINITIONS(-DBOOST_TEST_DYN_LINK)

find_package(ZCM REQUIRED)
find_package(Boost REQUIRED COMPONENTS unit_test_framework)

include_directories(
        ${ZCM_INCLUDE_DIRS}
        ${SW_INCLUDE_DIR}
        ${GNC_SRC_DIR}
        ${Boost_INCLUDE_DIRS}
        ${OPENCV_INCLUDE_DIR}
        ${EIGEN_INCLUDE_DIRS}
        ${SOPHUS_INCLUDE_DIR}
        ${YAML_CPP_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(testhelper SHARED TestHelpers.cpp)

target_link_libraries(testhelper
        maav-state
        maav-kalman
        ${ZCM_LIBRARIES})

set(TEST_SRCS
        StateTest.cpp
        HistoryTest.cpp
        UnscentedTransformTest.cpp
        PidTest.cpp
        LidarTest.cpp
        PredictionTest.cpp
        PlaneFitTest.cpp
        ZcmConversionTest.cpp
        GlobalUpdateTest.cpp
        MagnetometerTest.cpp
        PlannerUtilsTest.cpp)

foreach (testSrc ${TEST_SRCS})
    #Extract the filename without an extension (NAME_WE)
    get_filename_component(testName ${testSrc} NAME_WE)

    #Add compile target
    add_executable(${testName} ${testSrc})

    #link to Boost libraries AND your targets and dependencies
    target_link_libraries(${testName} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY}
            maav-state
            maav-measurements
            maav-gnc-utils
            maav-kalman
            maav-control
            maav-path-planner
            ${YAML_CPP_LIBRARY}
            yaml-cpp
            testhelper
            )

    set(TEST_BIN_DIR ${CMAKE_SOURCE_DIR}/bin/test)

    #I like to move testing binaries into a testBin directory
    set_target_properties(${testName} PROPERTIES
            RUNTIME_OUTPUT_DIRECTORY ${TEST_BIN_DIR})

    #Finally add it to test execution -
    #Notice the WORKING_DIRECTORY and COMMAND
    add_test(NAME ${testName}
            WORKING_DIRECTORY ${TEST_BIN_DIR}
            COMMAND ${TEST_BIN_DIR}/${testName})
endforeach (testSrc)

# copies over ground truth unit test values to testBin dir
set(GROUND_TRUTH_FILES
        pid_res.out
        controller_unit_test.yaml
        )

foreach(pid_res.out ${GROUND_TRUTH_FILES})
    configure_file(pid_res.out ${TEST_BIN_DIR}/pid_res.out COPYONLY)
endforeach(pid_res.out)
