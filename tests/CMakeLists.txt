if(NOT BUILD_TESTS)
    return()
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin/test)

add_subdirectory(gnc)
