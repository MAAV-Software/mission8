
if(NOT Octomap_FOUND)

    set(Octomap_FOUND true CACHE STRING "")

    set(Octomap_INCLUDE_DIRS "/usr/local/include/octomap")
    set(Octomap_INCLUDE_DIRS ${Octomap_INCLUDE_DIRS} CACHE STRING "")


    set(OctomapLibs /usr/local/lib/liboctomap.so
                    /usr/local/lib/liboctovis.so
                    /usr/local/lib/libdynamicedt3d.so
                    /usr/local/lib/liboctomath.so)

    # Find library
    set(Octomap_LIBRARIES ${OctomapLibs})
    set(Octomap_LIBRARIES ${Octomap_LIBRARIES} CACHE STRING "")

    mark_as_advanced(Octomap_INCLUDE_DIRS)
    mark_as_advanced(Octomap_LIBRARIES)

endif()
