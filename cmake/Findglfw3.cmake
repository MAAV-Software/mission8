# CMake find script for opengl
# Author: Adam Dziedzic


find_package(PkgConfig)

find_path(GLFW3_INCLUDE_DIR glfw3.h
	HINTS /usr/local/include /usr/include
	PATH_SUFFIXES GLFW)

find_library(GLFW3_LIBRARY libglfw.so
	HINTS /usr/lib /usr/local/lib
	PATH_SUFFIXES x86_64-linux-gnu)

set(GLFW3_LIBRARIES ${GLFW3_LIBRARY})

set(GLFW3_INCLUDE_DIRS ${GLFW3_INCLUDE_DIR})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GLFW3 DEFAULT_MSG
	GLFW3_LIBRARY
	GLFW3_INCLUDE_DIR)

mark_as_advanced(GLFW3_LIBRARY GLFW3_INCLUDE_DIR)
