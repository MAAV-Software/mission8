find_package(PkgConfig)
pkg_check_modules(PC_ZCM zcm)

find_path(ZCM_INCLUDE_DIR zcm
          HINTS ${PC_ZCM_INCLUDEDIR} ${PC_ZCM_INCLUDE_DIR}
          PATHS /usr/local/include /usr/include)

if( NOT ZCM_INCLUDE_DIR )
    message("-- Pkg-Config couldn't find it, so we're setting it to /usr/local/include/zcm")
    set(ZCM_INCLUDE_DIR /usr/local/include/zcm)
endif()

find_library(ZCM_LIBRARY zcm libzcm
             HINTS ${PC_ZCM_LIBDIR} ${PC_ZCM_LIBRARY_DIR} /usr/local/lib)

set(ZCM_LIBRARIES ${ZCM_LIBRARY})
set(ZCM_INCLUDE_DIRS ${ZCM_INCLUDE_DIR})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(zcm DEFAULT_MSG ZCM_LIBRARY ZCM_INCLUDE_DIR)
mark_as_advanced(ZCM_INCLUDE_DIR ZCM_LIBRARY)
