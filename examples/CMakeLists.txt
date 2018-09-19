if(NOT BUILD_EXAMPLES)
    return()
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin/examples)

add_subdirectory(gnc)
