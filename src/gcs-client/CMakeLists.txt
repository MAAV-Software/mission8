if(NOT BUILD_GCS_CLIENT)
    return()
endif()

find_package(ZCM REQUIRED)

include_directories(
    ${SW_INCLUDE_DIR}
	${YAMLCPP_INCLUDE_DIRS}
	${ZCM_INCLUDE_DIR}
)

add_executable(maav-gcs-client
	maav-gcs-client.cpp
)

target_link_libraries(maav-gcs-client
	maav-msg
    maav-utils
	${YAMLCPP_LIBRARY}
	${ZCM_LIBRARY}
)
