set(G2O_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/thirdparty/g2o)
set(DBOW_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/thirdparty/DBoW2)
set(SOPHUS_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/thirdparty/Sophus)

find_package(Eigen3 3.3.5 REQUIRED)

add_subdirectory(DBoW2)
add_subdirectory(g2o)
