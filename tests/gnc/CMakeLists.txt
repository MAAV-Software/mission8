project(tests)
enable_testing()
ADD_DEFINITIONS(-DBOOST_TEST_DYN_LINK)


find_package(Boost REQUIRED COMPONENTS unit_test_framework)

include_directories(
        ${SW_INCLUDE_DIR}
        ${GNC_SRC_DIR}
        ${Boost_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${SOPHUS_INCLUDE_DIR}
        ${YAML_CPP_INCLUDE_DIRS}
)

add_library(testhelper SHARED test_helpers.cpp)

target_link_libraries(testhelper
        maav-state
        maav-gnc
        maav-kalman)

set(TEST_SRCS
        state_test.cpp
        history_test.cpp
        unscented_transform_test.cpp
        pid_test.cpp)

foreach (testSrc ${TEST_SRCS})
    #Extract the filename without an extension (NAME_WE)
    get_filename_component(testName ${testSrc} NAME_WE)

    #Add compile target
    add_executable(${testName} ${testSrc})

    #link to Boost libraries AND your targets and dependencies
    target_link_libraries(${testName} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY}
            maav-state
            maav-gnc
            maav-kalman
            maav-control
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
