set(OpenCV_FOUND FALSE)


# Find include dirs

if(NOT OpenCV_INCLUDE_DIR)
    find_path(OpenCV_INCLUDE_DIR opencv.hpp
        PATH_SUFFIXES thirdparty thirdparty/opencv thirdparty/opencv/include
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
if(NOT OpenCV_LIBRARY)
    find_library(OpenCV_LIBRARY NAMES cv cv2 opencv opencv2
        PATH_SUFFIXES thirdparty thirdparty/opencv thirdparty/opencv/bin
        PATHS ${PROJECT_SOURCE_DIR}/thirdparty

    )
endif()
