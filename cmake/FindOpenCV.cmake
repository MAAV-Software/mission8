# find_package(PkgConfig)
# pkg_check_modules(PC_OPENCV opencv)

# Find include dirs

if(NOT OpenCV_INCLUDE_DIR)
    find_path(OpenCV_INCLUDE_DIR opencv2/core.hpp
        PATH_SUFFIXES opencv opencv/include
        PATHS ${SOFTWARE_SOURCE_DIR}/thirdparty
    )
	SET(OPENCV_VERSION_FILE "${SOFTWARE_SOURCE_DIR}/thirdparty/opencv/include/opencv2/core/version.hpp")
	file(STRINGS "${OPENCV_VERSION_FILE}" OPENCV_VERSION_PARTS REGEX "#define CV_VERSION_[A-Z]+[ ]+" )

	string(REGEX REPLACE ".+CV_VERSION_MAJOR[ ]+([0-9]+).*" "\\1" OPENCV_VERSION_MAJOR "${OPENCV_VERSION_PARTS}")
	string(REGEX REPLACE ".+CV_VERSION_MINOR[ ]+([0-9]+).*" "\\1" OPENCV_VERSION_MINOR "${OPENCV_VERSION_PARTS}")
	string(REGEX REPLACE ".+CV_VERSION_REVISION[ ]+([0-9]+).*" "\\1" OPENCV_VERSION_PATCH "${OPENCV_VERSION_PARTS}")
	string(REGEX REPLACE ".+CV_VERSION_STATUS[ ]+\"([^\"]*)\".*" "\\1" OPENCV_VERSION_STATUS "${OPENCV_VERSION_PARTS}")

	set(OPENCV_VERSION_PLAIN "${OPENCV_VERSION_MAJOR}.${OPENCV_VERSION_MINOR}.${OPENCV_VERSION_PATCH}")

	set(OPENCV_VERSION "${OPENCV_VERSION_PLAIN}${OPENCV_VERSION_STATUS}")

	set(OPENCV_SOVERSION "${OPENCV_VERSION_MAJOR}.${OPENCV_VERSION_MINOR}")
	set(OPENCV_LIBVERSION "${OPENCV_VERSION_MAJOR}.${OPENCV_VERSION_MINOR}.${OPENCV_VERSION_PATCH}")

	set(OpenCV_FOUND true CACHE STRING "")

	set(OPENCV_INCLUDE_DIR ${OpenCV_INCLUDE_DIR} CACHE STRING "")
	set(OPENCV_INCLUDE_DIRS ${OpenCV_INCLUDE_DIR} CACHE STRING "")
	set(OpenCV_INCLUDE_DIRS ${OpenCV_INCLUDE_DIR} CACHE STRING "")
	mark_as_advanced(OPENCV_INCLUDE_DIR OPENCV_INCLUDE_DIRS OpenCV_INCLUDE_DIR OpenCV_INCLUDE_DIRS)

	message("OPENCV INCLUDE DIR: ${OpenCV_INCLUDE_DIRS}")
	# create a dependency on the version file
	# we never use the output of the following command but cmake will rerun automatically if the version file changes
	configure_file("${OPENCV_VERSION_FILE}" "${CMAKE_BINARY_DIR}/junk/version.junk" COPYONLY)
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
        PATH_SUFFIXES opencv opencv/lib
        PATHS ${SOFTWARE_SOURCE_DIR}/thirdparty
    )

    find_library(OpenCV_BGSEGM_LIBRARY NAMES opencv_bgsegm libopencv_bgsegm
        PATH_SUFFIXES opencv opencv/lib
        PATHS ${SOFTWARE_SOURCE_DIR}/thirdparty
    )

    find_library(OpenCV_BIOINSPIRED_LIBRARY
        NAMES opencv_bioinspired libopencv_bilibopencv_bioinspired
        PATH_SUFFIXES opencv opencv/lib
        PATHS ${SOFTWARE_SOURCE_DIR}/thirdparty
    )

    find_library(OpenCV_CALIB3D_LIBRARY NAMES opencv_calib3d libopencv_calib3d
        PATH_SUFFIXES opencv opencv/lib
        PATHS ${SOFTWARE_SOURCE_DIR}/thirdparty
    )

find_library(OpenCV_CCALIB_LIBRARY NAMES opencv_ccalib libopencv_ccalib
        PATH_SUFFIXES opencv opencv/lib
        PATHS ${SOFTWARE_SOURCE_DIR}/thirdparty
    )

    find_library(OpenCV_CORE_LIBRARY NAMES opencv_core libopencv_core
        PATH_SUFFIXES opencv opencv/lib
        PATHS ${SOFTWARE_SOURCE_DIR}/thirdparty
    )

find_library(OpenCV_CUDAARITHM_LIBRARY NAMES opencv_cudaarithm libopencv_cudaarithm
        PATH_SUFFIXES opencv opencv/lib
        PATHS ${SOFTWARE_SOURCE_DIR}/thirdparty
    )

find_library(OpenCV_CUDABGSEGM_LIBRARY NAMES opencv_cudabgsegm libopencv_cudabgsegm
        PATH_SUFFIXES opencv opencv/lib
        PATHS ${SOFTWARE_SOURCE_DIR}/thirdparty
    )

find_library(OpenCV_CUDACODEC_LIBRARY NAMES opencv_cudacodec libopencv_cudacodec
        PATH_SUFFIXES opencv opencv/lib
        PATHS ${SOFTWARE_SOURCE_DIR}/thirdparty
    )

find_library(OpenCV_CUDAFEATURES2D_LIBRARY NAMES opencv_cudafeatures2d libopencv_cudafeatures2d
        PATH_SUFFIXES opencv opencv/lib
        PATHS ${SOFTWARE_SOURCE_DIR}/thirdparty
    )

find_library(OpenCV_CUDAFILTERS_LIBRARY NAMES opencv_cudafilters libopencv_cudafilters
        PATH_SUFFIXES opencv opencv/lib
        PATHS ${SOFTWARE_SOURCE_DIR}/thirdparty
    )

find_library(OpenCV_CUDAIMGPROC_LIBRARY NAMES opencv_cudaimgproc libopencv_cudaimgproc
        PATH_SUFFIXES opencv opencv/lib
        PATHS ${SOFTWARE_SOURCE_DIR}/thirdparty
    )

find_library(OpenCV_CUDALEGACY_LIBRARY NAMES opencv_cudalegacy libopencv_cudalegacy
        PATH_SUFFIXES opencv opencv/lib
        PATHS ${SOFTWARE_SOURCE_DIR}/thirdparty
    )

find_library(OpenCV_CUDAOBJDETECT_LIBRARY NAMES opencv_cudaobjdetect libopencv_cudaobjdetect
        PATH_SUFFIXES opencv opencv/lib
        PATHS ${SOFTWARE_SOURCE_DIR}/thirdparty
    )

find_library(OpenCV_CUDAOPTFLOW_LIBRARY NAMES opencv_cudaoptflow libopencv_cudaoptflow
        PATH_SUFFIXES opencv opencv/lib
        PATHS ${SOFTWARE_SOURCE_DIR}/thirdparty
    )

find_library(OpenCV_CUDASTEREO_LIBRARY NAMES opencv_cudastereo libopencv_cudastereo
        PATH_SUFFIXES opencv opencv/lib
        PATHS ${SOFTWARE_SOURCE_DIR}/thirdparty
    )

find_library(OpenCV_CUDAWARPING_LIBRARY NAMES opencv_cudawarping libopencv_cudawarping
        PATH_SUFFIXES opencv opencv/lib
        PATHS ${SOFTWARE_SOURCE_DIR}/thirdparty
    )

find_library(OpenCV_CUDEV_LIBRARY NAMES opencv_cudev libopencv_cudev
        PATH_SUFFIXES opencv opencv/lib
        PATHS ${SOFTWARE_SOURCE_DIR}/thirdparty
    )

find_library(OpenCV_DATASETS_LIBRARY NAMES opencv_datasets libopencv_datasets
        PATH_SUFFIXES opencv opencv/lib
        PATHS ${SOFTWARE_SOURCE_DIR}/thirdparty
    )

find_library(OpenCV_DNN_LIBRARY NAMES opencv_dnn libopencv_dnn
        PATH_SUFFIXES opencv opencv/lib
        PATHS ${SOFTWARE_SOURCE_DIR}/thirdparty
    )

find_library(OpenCV_DPM_LIBRARY NAMES opencv_dpm libopencv_dpm
        PATH_SUFFIXES opencv opencv/lib
        PATHS ${SOFTWARE_SOURCE_DIR}/thirdparty
    )

find_library(OpenCV_FACE_LIBRARY NAMES opencv_features2d libopencv_features2d
        PATH_SUFFIXES opencv opencv/lib
        PATHS ${SOFTWARE_SOURCE_DIR}/thirdparty
    )

find_library(OpenCV_FEATURES2d_LIBRARY
        NAMES opencv_features2d libopencv_features2d
        PATH_SUFFIXES opencv opencv/lib
        PATHS ${SOFTWARE_SOURCE_DIR}/thirdparty
    )

find_library(OpenCV_FLANN_LIBRARY NAMES opencv_flann libopencv_flann
        PATH_SUFFIXES opencv opencv/lib
        PATHS ${SOFTWARE_SOURCE_DIR}/thirdparty
    )

find_library(OpenCV_FREETYPE_LIBRARY NAMES opencv_freetype libopencv_freetype
        PATH_SUFFIXES opencv opencv/lib
        PATHS ${SOFTWARE_SOURCE_DIR}/thirdparty
    )

find_library(OpenCV_FUZZY_LIBRARY NAMES opencv_fuzzy libopencv_fuzzy
        PATH_SUFFIXES opencv opencv/lib
        PATHS ${SOFTWARE_SOURCE_DIR}/thirdparty
    )

find_library(OpenCV_HDF_LIBRARY NAMES opencv_hdf libopencv_hdf
        PATH_SUFFIXES opencv opencv/lib
        PATHS ${SOFTWARE_SOURCE_DIR}/thirdparty
    )

find_library(OpenCV_HIGHGUI_LIBRARY NAMES opencv_highgui libopencv_highgui
        PATH_SUFFIXES opencv opencv/lib
        PATHS ${SOFTWARE_SOURCE_DIR}/thirdparty
    )

find_library(OpenCV_IMGCODECS_LIBRARY NAMES opencv_imgcodecs libopencv_imgcodecs
        PATH_SUFFIXES opencv opencv/lib
        PATHS ${SOFTWARE_SOURCE_DIR}/thirdparty
    )

find_library(OpenCV_IMG_HASH_LIBRARY NAMES opencv_img_hash libopencv_img_hash
        PATH_SUFFIXES opencv opencv/lib
        PATHS ${SOFTWARE_SOURCE_DIR}/thirdparty
    )

find_library(OpenCV_IMGPROC_LIBRARY NAMES opencv_imgproc libopencv_imgproc
        PATH_SUFFIXES opencv opencv/lib
        PATHS ${SOFTWARE_SOURCE_DIR}/thirdparty
    )

find_library(OpenCV_LINE_DESCRIPTOR_LIBRARY
        NAMES opencv_line_descriptor libopencv_line_descriptor
        PATH_SUFFIXES opencv opencv/lib
        PATHS ${SOFTWARE_SOURCE_DIR}/thirdparty
    )

    find_library(OpenCV_ML_LIBRARY NAMES opencv_ml libopencv_ml
        PATH_SUFFIXES opencv opencv/lib
        PATHS ${SOFTWARE_SOURCE_DIR}/thirdparty
    )

find_library(OpenCV_OBJDETECT_LIBRARY NAMES opencv_objdetect libopencv_objdetect
        PATH_SUFFIXES opencv opencv/lib
        PATHS ${SOFTWARE_SOURCE_DIR}/thirdparty
    )

find_library(OpenCV_OPTFLOW_LIBRARY NAMES opencv_optflow libopencv_optflow
        PATH_SUFFIXES opencv opencv/lib
        PATHS ${SOFTWARE_SOURCE_DIR}/thirdparty
    )


find_library(OpenCV_PHASE_UNWRAPPING_LIBRARY NAMES opencv_phase_unwrapping libopencv_phase_unwrapping
        PATH_SUFFIXES opencv opencv/lib
        PATHS ${SOFTWARE_SOURCE_DIR}/thirdparty
    )

find_library(OpenCV_PHOTO_LIBRARY NAMES opencv_photo libopencv_photo
        PATH_SUFFIXES opencv opencv/lib
        PATHS ${SOFTWARE_SOURCE_DIR}/thirdparty
    )

find_library(OpenCV_PLOT_LIBRARY NAMES opencv_plot libopencv_plot
        PATH_SUFFIXES opencv opencv/lib
        PATHS ${SOFTWARE_SOURCE_DIR}/thirdparty
    )

find_library(OpenCV_REG_LIBRARY NAMES opencv_reg libopencv_reg
        PATH_SUFFIXES opencv opencv/lib
        PATHS ${SOFTWARE_SOURCE_DIR}/thirdparty
    )

find_library(OpenCV_RGBD_LIBRARY NAMES opencv_rgbd libopencv_rgdb
        PATH_SUFFIXES opencv opencv/lib
        PATHS ${SOFTWARE_SOURCE_DIR}/thirdparty
    )

find_library(OpenCV_SALIENCY_LIBRARY NAMES opencv_saliency libopencv_saliency
        PATH_SUFFIXES opencv opencv/lib
        PATHS ${SOFTWARE_SOURCE_DIR}/thirdparty
    )

find_library(OpenCV_SALIENCY_LIBRARY NAMES opencv_saliency libopencv_saliency
        PATH_SUFFIXES opencv opencv/lib
        PATHS ${SOFTWARE_SOURCE_DIR}/thirdparty
    )

find_library(OpenCV_SHAPE_LIBRARY NAMES opencv_shape libopencv_shape
        PATH_SUFFIXES opencv opencv/lib
        PATHS ${SOFTWARE_SOURCE_DIR}/thirdparty
    )

find_library(OpenCV_STEREO_LIBRARY NAMES opencv_stereo libopencv_stereo
        PATH_SUFFIXES opencv opencv/lib
        PATHS ${SOFTWARE_SOURCE_DIR}/thirdparty
    )


find_library(OpenCV_STITCHING_LIBRARY NAMES opencv_stitching libopencv_stitching
        PATH_SUFFIXES opencv opencv/lib
        PATHS ${SOFTWARE_SOURCE_DIR}/thirdparty
    )

find_library(OpenCV_STRUCTURED_LIGHT_LIBRARY NAMES opencv_structured_light libopencv_structured_light
        PATH_SUFFIXES opencv opencv/lib
        PATHS ${SOFTWARE_SOURCE_DIR}/thirdparty
    )

find_library(OpenCV_SUPERRES_LIBRARY NAMES opencv_superres libopencv_superres
        PATH_SUFFIXES opencv opencv/lib
        PATHS ${SOFTWARE_SOURCE_DIR}/thirdparty
    )

find_library(OpenCV_SURFACE_MATCHING_LIBRARY NAMES opencv_surface_matching libopencv_surface_matching
        PATH_SUFFIXES opencv opencv/lib
        PATHS ${SOFTWARE_SOURCE_DIR}/thirdparty
    )

find_library(OpenCV_TEXT_LIBRARY NAMES opencv_text libopencv_text
        PATH_SUFFIXES opencv opencv/lib
        PATHS ${SOFTWARE_SOURCE_DIR}/thirdparty
    )

find_library(OpenCV_TRACKING_LIBRARY NAMES opencv_tracking libopencv_tracking
        PATH_SUFFIXES opencv opencv/lib
        PATHS ${SOFTWARE_SOURCE_DIR}/thirdparty
    )

find_library(OpenCV_VIDEOIO_LIBRARY NAMES opencv_videoio libopencv_videoio
        PATH_SUFFIXES opencv opencv/lib
        PATHS ${SOFTWARE_SOURCE_DIR}/thirdparty
    )

find_library(OpenCV_VIDEO_LIBRARY NAMES opencv_video libopencv_video
        PATH_SUFFIXES opencv opencv/lib
        PATHS ${SOFTWARE_SOURCE_DIR}/thirdparty
    )

find_library(OpenCV_VIDEOSTAB_LIBRARY NAMES opencv_videostab libopencv_videostab
        PATH_SUFFIXES opencv opencv/lib
        PATHS ${SOFTWARE_SOURCE_DIR}/thirdparty
    )

find_library(OpenCV_VIZ_LIBRARY NAMES opencv_viz libopencv_viz
        PATH_SUFFIXES opencv opencv/lib
        PATHS ${SOFTWARE_SOURCE_DIR}/thirdparty
    )

find_library(OpenCV_XFEATURES2D_LIBRARY NAMES opencv_xfeatures2d libopencv_xfeatures2d
        PATH_SUFFIXES opencv opencv/lib
        PATHS ${SOFTWARE_SOURCE_DIR}/thirdparty
    )

find_library(OpenCV_XIMGPROC_LIBRARY NAMES opencv_ximgproc libopencv_ximgproc
        PATH_SUFFIXES opencv opencv/lib
        PATHS ${SOFTWARE_SOURCE_DIR}/thirdparty
    )

find_library(OpenCV_XOBJDETECT_LIBRARY NAMES opencv_xobjdetect libopencv_xobjdetect
        PATH_SUFFIXES opencv opencv/lib
        PATHS ${SOFTWARE_SOURCE_DIR}/thirdparty
    )

find_library(OpenCV_XPHOTO_LIBRARY NAMES opencv_xphoto libopencv_xphoto
        PATH_SUFFIXES opencv opencv/lib
        PATHS ${SOFTWARE_SOURCE_DIR}/thirdparty
    )

set(OPENCV_LIBRARIES
    ${OpenCV_ARUCO_LIBRARY}
    ${OpenCV_BGSEGM_LIBRARY}
    ${OpenCV_CALIB3D_LIBRARY}
    ${OpenCV_CALIB_LIBRARY}
    ${OpenCV_CORE_LIBRARY}
    ${OpenCV_DATASETS_LIBRARY}
    ${OpenCV_DNN_LIBRARY}
    ${OpenCV_DPM_LIBRARY}
    ${OpenCV_FACE_LIBRARY}
    ${OpenCV_FLANN_LIBRARY}
    ${OpenCV_FREETYPE_LIBRARY}
    ${OpenCV_FUZZY_LIBRARY}
    ${OpenCV_HDF_LIBRARY}
    ${OpenCV_HIGHGUI_LIBRARY}
    ${OpenCV_IMGCODECS_LIBRARY}
    ${OpenCV_IMG_HASH_LIBRARY}
    ${OpenCV_IMGPROC_LIBRARY}
    ${OpenCV_ML_LIBRARY}
    ${OpenCV_OBJDETECT_LIBRARY}
    ${OpenCV_OPTFLOW_LIBRARY}
    ${OpenCV_PHASE_UNWRAPPING_LIBRARY}
    ${OpenCV_PHOTO_LIBRARY}
    ${OpenCV_PLOT_LIBRARY}
    ${OpenCV_REG_LIBRARY}
    ${OpenCV_RGBD_LIBRARY}
    ${OpenCV_SALIENCY_LIBRARY}
    ${OpenCV_SALIENCY_LIBRARY}
    ${OpenCV_SHAPE_LIBRARY}
    ${OpenCV_STEREO_LIBRARY}
    ${OpenCV_STITCHING_LIBRARY}
    ${OpenCV_STRUCTURED_LIGHT_LIBRARY}
    ${OpenCV_SUPERRES_LIBRARY}
    ${OpenCV_SURFACE_MATCHING_LIBRARY}
    ${OpenCV_TEXT_LIBRARY}
    ${OpenCV_TRACKING_LIBRARY}
    ${OpenCV_VIDEOIO_LIBRARY}
    ${OpenCV_VIDEO_LIBRARY}
    ${OpenCV_VIDEOSTAB_LIBRARY}
    ${OpenCV_VIZ_LIBRARY}
    ${OpenCV_XFEATURES2D_LIBRARY}
    ${OpenCV_XIMPROC_LIBRARY}
    ${OpenCV_XOBJDETECT_LIBRARY}
    ${OpenCV_XPHOTO_LIBRARY}
    ${OpenCV_ARUCO_LIBRARY}
    ${OpenCV_BGSEGM_LIBRARY}
    ${OpenCV_CALIB3D_LIBRARY}
    ${OpenCV_CALIB_LIBRARY}
    ${OpenCV_CORE_LIBRARY}
    ${OpenCV_DATASETS_LIBRARY}
    ${OpenCV_DNN_LIBRARY}
    ${OpenCV_DPM_LIBRARY}
    ${OpenCV_FACE_LIBRARY}
    ${OpenCV_FLANN_LIBRARY}
    ${OpenCV_FREETYPE_LIBRARY}
    ${OpenCV_FUZZY_LIBRARY}
    ${OpenCV_HDF_LIBRARY}
    ${OpenCV_HIGHGUI_LIBRARY}
    ${OpenCV_IMGCODECS_LIBRARY}
    ${OpenCV_IMG_HASH_LIBRARY}
    ${OpenCV_IMGPROC_LIBRARY}
    ${OpenCV_ML_LIBRARY}
    ${OpenCV_OBJDETECT_LIBRARY}
    ${OpenCV_OPTFLOW_LIBRARY}
    ${OpenCV_PHASE_UNWRAPPING_LIBRARY}
    ${OpenCV_PHOTO_LIBRARY}
    ${OpenCV_PLOT_LIBRARY}
    ${OpenCV_REG_LIBRARY}
    ${OpenCV_RGBD_LIBRARY}
    ${OpenCV_SALIENCY_LIBRARY}
    ${OpenCV_SALIENCY_LIBRARY}
    ${OpenCV_SHAPE_LIBRARY}
    ${OpenCV_STEREO_LIBRARY}
    ${OpenCV_STITCHING_LIBRARY}
    ${OpenCV_STRUCTURED_LIGHT_LIBRARY}
    ${OpenCV_SUPERRES_LIBRARY}
    ${OpenCV_SURFACE_MATCHING_LIBRARY}
    ${OpenCV_TEXT_LIBRARY}
    ${OpenCV_TRACKING_LIBRARY}
    ${OpenCV_VIDEOIO_LIBRARY}
    ${OpenCV_VIDEO_LIBRARY}
    ${OpenCV_VIDEOSTAB_LIBRARY}
    ${OpenCV_VIZ_LIBRARY}
    ${OpenCV_XFEATURES2D_LIBRARY}
    ${OpenCV_XIMPROC_LIBRARY}
    ${OpenCV_XOBJDETECT_LIBRARY}
    ${OpenCV_XPHOTO_LIBRARY}
	CACHE STRING ""
)

set(OpenCV_LIBS ${OPENCV_LIBRARIES} CACHE STRING "")

mark_as_advanced(OpenCV_LIBS)
mark_as_advanced(OPENCV_LIBRARIES)

endif()
