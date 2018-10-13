set (OPENCV_INCLUDE_DIRS)

# Find include dirs

if(NOT OpenCV_INCLUDE_DIR)
	message("spooky meatball")
    find_path(OpenCV_INCLUDE_DIR opencv/cv.h opencv/cv.hpp
        PATH_SUFFIXES opencv opencv/include
        PATHS ${SOFTWARE_SOURCE_DIR}/thirdparty
    )
	set(OpenCV_THIRD_PARTY true)
	set(OpenCV_FOUND true)
	set(OpenCV_VERSION 3.3.1)
endif()

# Find libraries

if(OpenCV_THIRD_PARTY)
	message("retarded meatball")
	file(GLOB OpenCV_LIBRARIES ${SOFTWARE_SOURCE_DIR}/thirdparty/opencv/lib/*.so)
else()
	find_package(OpenCV)
endif()
