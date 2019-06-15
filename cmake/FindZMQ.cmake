find_package(PkgConfig)
pkg_check_modules(PC_ZMQ QUIET zmq)

find_path(ZMQ_INCLUDE_DIR zmq.hpp
          HINTS ${PC_ZMQ_INCLUDEDIR} ${PC_ZMQ_INCLUDE_DIR}
          PATHS /usr/local/include /usr/include)

if( NOT ZMQ_INCLUDE_DIR )
    message("-- Pkg-Config couldn't find it, so we're setting it to /usr/local/include/zmq")
    set(ZCM_INCLUDE_DIR /usr/local/include/zcm)
endif()

find_library(ZMQ_LIBRARY zmq libzmq libczmq
             HINTS ${PC_ZMQ_LIBDIR} ${PC_ZMQ_LIBRARY_DIR} /usr/local/lib)

set(ZMQ_LIBRARIES ${ZMQ_LIBRARY})
set(ZMQ_INCLUDE_DIRS ${ZMQ_INCLUDE_DIR})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(zmq DEFAULT_MSG ZMQ_LIBRARY ZMQ_INCLUDE_DIR)
mark_as_advanced(ZMQ_INCLUDE_DIR ZMQ_LIBRARY)
