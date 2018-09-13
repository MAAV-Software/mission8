get_filename_component(SELF_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)

find_package (Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(OpenCV 3.0 QUIET)

set(MAAV_GNC_INCLUDE_DIRS 
    ${SELF_DIR}/include
    ${SELF_DIR}/Thirdparty/g2o
    ${SELF_DIR}/Thirdparty/DBoW2
    ${SELF_DIR}/Thirdparty/Sophus
    ${Pangolin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

set(MAAV_GNC_LIBRARY_DIRS ${SELF_DIR}/lib)

set(MAAV_GNC_LIBRARIES 
    ${MAAV_GNC_LIBRARY_DIRS}/libmaav-state.so
    ${MAAV_GNC_LIBRARY_DIRS}/libmaav-slam.so
    ${MAAV_GNC_LIBRARY_DIRS}/libmaav-kalman.so
    ${MAAV_GNC_LIBRARY_DIRS}/libmaav-gnc.so
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
    ${Pangolin_LIBRARIES}
)