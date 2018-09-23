find_package(PkgConfig)
pkg_check_modules(PC_ZCM zcm)

# Find include dirs

if(NOT OpenCV_INCLUDE_DIR)
    find_path(OpenCV_INCLUDE_DIR opencv/cv.h opencv/cv.hpp
        PATH_SUFFIXES opencv opencv/include
        PATHS ${PROJECT_SOURCE_DIR}/thirdparty
    )
endif()

if(NOT OpenCV_INCLUDE_DIR)
    message("-- Could not find OpenCV in thirdparty, looking in system dirs")
    find_path(OpenCV_INCLUDE_DIR opencv.hpp
        PATH_SUFFIXES opencv2
        PATHS /usr/local/include /usr/include
    )
endif()

# Find library
if(NOT OpenCV_LIBRARIES)
    find_library(OpenCV_ARUCO_LIBRARY NAMES opencv_aruco libopencv_aruco
        PATH_SUFFIXES opencv opencv/bin
        PATHS ${PROJECT_SOURCE_DIR}/thirdparty
    )

    find_library(OpenCV_BGSEGM_LIBRARY NAMES opencv_bgsegm libopencv_bgsegm
        PATH_SUFFIXES opencv opencv/bin
        PATHS ${PROJECT_SOURCE_DIR}/thirdparty
    )

    find_library(OpenCV_BIOINSPIRED_LIBRARY
        NAMES opencv_bioinspired libopencv_bilibopencv_bioinspired
        PATH_SUFFIXES opencv opencv/bin
        PATHS ${PROJECT_SOURCE_DIR}/thirdparty
    )

    find_library(OpenCV_CALIB3D_LIBRARY NAMES opencv_calib3d libopencv_calib3d
        PATH_SUFFIXES opencv opencv/bin
        PATHS ${PROJECT_SOURCE_DIR}/thirdparty
    )

    find_library(OpenCV_CALIB_LIBRARY NAMES opencv_calib libopencv_calib
        PATH_SUFFIXES opencv opencv/bin
        PATHS ${PROJECT_SOURCE_DIR}/thirdparty
    )

    find_library(OpenCV_CORE_LIBRARY NAMES opencv_core libopencv_core
        PATH_SUFFIXES opencv opencv/bin
        PATHS ${PROJECT_SOURCE_DIR}/thirdparty
    )

find_library(OpenCV_DATASETS_LIBRARY NAMES opencv_datasets libopencv_datasets
        PATH_SUFFIXES opencv opencv/bin
        PATHS ${PROJECT_SOURCE_DIR}/thirdparty
    )

find_library(OpenCV_DNN_LIBRARY NAMES opencv_dnn libopencv_dnn
        PATH_SUFFIXES opencv opencv/bin
        PATHS ${PROJECT_SOURCE_DIR}/thirdparty
    )

find_library(OpenCV_DPM_LIBRARY NAMES opencv_dpm libopencv_dpm
        PATH_SUFFIXES opencv opencv/bin
        PATHS ${PROJECT_SOURCE_DIR}/thirdparty
    )

find_library(OpenCV_FACE_LIBRARY NAMES opencv_features2d libopencv_features2d
        PATH_SUFFIXES opencv opencv/bin
        PATHS ${PROJECT_SOURCE_DIR}/thirdparty
    )

find_library(OpenCV_FEATURES2d_LIBRARY
        NAMES opencv_features2d libopencv_features2d
        PATH_SUFFIXES opencv opencv/bin
        PATHS ${PROJECT_SOURCE_DIR}/thirdparty
    )

find_library(OpenCV_FLANN_LIBRARY NAMES opencv_flann libopencv_flann
        PATH_SUFFIXES opencv opencv/bin
        PATHS ${PROJECT_SOURCE_DIR}/thirdparty
    )

find_library(OpenCV_FREETYPE_LIBRARY NAMES opencv_freetype libopencv_freetype
        PATH_SUFFIXES opencv opencv/bin
        PATHS ${PROJECT_SOURCE_DIR}/thirdparty
    )

find_library(OpenCV_FUZZY_LIBRARY NAMES opencv_fuzzy libopencv_fuzzy
        PATH_SUFFIXES opencv opencv/bin
        PATHS ${PROJECT_SOURCE_DIR}/thirdparty
    )

find_library(OpenCV_HDF_LIBRARY NAMES opencv_hdf libopencv_hdf
        PATH_SUFFIXES opencv opencv/bin
        PATHS ${PROJECT_SOURCE_DIR}/thirdparty
    )

find_library(OpenCV_HIGHGUI_LIBRARY NAMES opencv_highgui libopencv_highgui
        PATH_SUFFIXES opencv opencv/bin
        PATHS ${PROJECT_SOURCE_DIR}/thirdparty
    )

find_library(OpenCV_IMGCODECS_LIBRARY NAMES opencv_imgcodecs libopencv_imgcodecs
        PATH_SUFFIXES opencv opencv/bin
        PATHS ${PROJECT_SOURCE_DIR}/thirdparty
    )

find_library(OpenCV_IMG_HASH_LIBRARY NAMES opencv_img_hash libopencv_img_hash
        PATH_SUFFIXES opencv opencv/bin
        PATHS ${PROJECT_SOURCE_DIR}/thirdparty
    )

find_library(OpenCV_IMGPROC_LIBRARY NAMES opencv_imgproc libopencv_imgproc
        PATH_SUFFIXES opencv opencv/bin
        PATHS ${PROJECT_SOURCE_DIR}/thirdparty
    )

find_library(OpenCV_LINE_DESCRIPTOR_LIBRARY
        NAMES opencv_lline_descriptor libopencv_line_descriptor
        PATH_SUFFIXES opencv opencv/bin
        PATHS ${PROJECT_SOURCE_DIR}/thirdparty
    )

    find_library(OpenCV_ML_LIBRARY NAMES opencv_ml libopencv_ml
        PATH_SUFFIXES opencv opencv/bin
        PATHS ${PROJECT_SOURCE_DIR}/thirdparty
    )

find_library(OpenCV_OBJDETECT_LIBRARY NAMES opencv_objdetect libopencv_objdetect
        PATH_SUFFIXES opencv opencv/bin
        PATHS ${PROJECT_SOURCE_DIR}/thirdparty
    )

endif()
