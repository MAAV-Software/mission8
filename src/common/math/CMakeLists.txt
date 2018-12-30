find_package(OpenCV REQUIRED)

include_directories(
	${OpenCV_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${YAML_INCLUDE_DIRS}
)

add_library(maav-math SHARED
    angle_functions.cpp
    FieldOfView.cpp
    FORMapper.cpp
    math.cpp
    math_utils.cpp
    LowPass.cpp
)

target_link_libraries(maav-math
    ${YAML_LIBRARY}
	${OpenCV_LIBS}
)