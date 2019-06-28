# if (BUILD_PLOTTER)
#     add_subdirectory(plotter)
# endif()

# if(BUILD_VISUALIZER)
#     add_subdirectory(localization_viz)
# endif()

if (BUILD_GNC AND BUILD_VISION)
    add_subdirectory(octomap)
    # add_subdirectory(octomap_viz)
    add_subdirectory(planner)
endif()