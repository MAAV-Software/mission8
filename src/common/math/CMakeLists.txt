include_directories(
    ${EIGEN3_INCLUDE_DIR}
    ${YAML_INCLUDE_DIRS}
)

add_library(maav-math SHARED
    angle_functions.cpp
    FieldOfView.cpp
    FORMapper.cpp
    math.cpp
    math_utils.cpp
)

target_link_libraries(maav-math
    ${YAML_LIBRARY}
)
