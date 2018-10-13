# Find commonly used packages
find_package(Eigen3 3.3.5)
if(NOT EIGEN3_VERSION_OK)
    message("You don't have the eigen submodule, prepare for a bad time.")
    message("Don't compile with -Werror.")
    message("Looking for an older version of Eigen (3.3.5 recommended)")
    find_package(Eigen3 REQUIRED)
endif()

# MAAV's include directory
include_directories(
    ${SW_INCLUDE_DIR}
)

add_subdirectory(common)
add_subdirectory(gcs-client)
add_subdirectory(gnc)
add_subdirectory(lidar)
add_subdirectory(obstacles)
add_subdirectory(tanfan)
add_subdirectory(vision)
