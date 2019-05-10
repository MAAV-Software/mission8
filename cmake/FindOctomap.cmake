
if(NOT Octomap_FOUND)

    set(Octomap_FOUND true CACHE STRING "")

    set(Octomap_INCLUDE_DIRS "${SOFTWARE_SOURCE_DIR}/thirdparty/octomap/include")
    set(Octomap_INCLUDE_DIRS ${Octomap_INCLUDE_DIRS} CACHE STRING "")


    set(OctomapLibs ${SOFTWARE_SOURCE_DIR}/thirdparty/octomap/lib/liboctomap.so
                                        ${SOFTWARE_SOURCE_DIR}/thirdparty/octomap/lib/liboctovis.so
                                        ${SOFTWARE_SOURCE_DIR}/thirdparty/octomap/lib/libdynamicedt3d.so
                                        ${SOFTWARE_SOURCE_DIR}/thirdparty/octomap/lib/liboctomath.so)

    # Find library
    set(Octomap_LIBRARIES ${OctomapLibs})
    set(Octomap_LIBRARIES ${Octomap_LIBRARIES} CACHE STRING "")

    mark_as_advanced(Octomap_INCLUDE_DIRS)
    mark_as_advanced(Octomap_LIBRARIES)

endif()
