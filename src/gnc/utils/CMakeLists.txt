include_directories(maav-gnc-utils
    ${SW_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${SOPHUS_INCLUDE_DIR}
)

add_library(maav-gnc-utils SHARED
    ZcmConversion.cpp
)