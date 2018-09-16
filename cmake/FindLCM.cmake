find_package(PkgConfig)
pkg_check_modules(PC_LCM lcm)

find_path(LCM_INCLUDE_DIR lcm
          HINTS ${PC_LCM_INCLUDEDIR} ${PC_LCM_INCLUDE_DIR}
          PATHS /usr/local/include /usr/include)

if( NOT LCM_INCLUDE_DIR )
    MESSAGE("-- Pkg-Config couldn't find it, so we're setting it to /usr/local/include/lcm")
    set(LCM_INCLUDE_DIR /usr/local/include/lcm)
endif()

find_library(LCM_LIBRARY lcm liblcm
             HINTS ${PC_LCM_LIBDIR} ${PC_LCM_LIBRARY_DIR} /usr/local/lib)

set(LCM_LIBRARIES ${LCM_LIBRARY})
set(LCM_INCLUDE_DIRS ${LCM_INCLUDE_DIR})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(lcm DEFAULT_MSG LCM_LIBRARY LCM_INCLUDE_DIR)
mark_as_advanced(LCM_INCLUDE_DIR LCM_LIBRARY)
