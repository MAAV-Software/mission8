find_package(Octomap REQUIRED)

add_library(maav-path-planner SHARED
    Astar.cpp
)

target_include_directories(maav-path-planner PUBLIC
    ${SW_INCLUDE_DIR}
)

target_include_directories(maav-path-planner SYSTEM PUBLIC
${EIGEN_INCLUDE_DIRS}
${Octomap_INCLUDE_DIRS}
)

target_link_libraries(maav-path-planner
    maav-state
    maav-measurements
    ${EIGEN3_LIBS}
    ${Octomap_LIBRARIES}
)