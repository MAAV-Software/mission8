if (BUILD_PLOTTER AND BUILD_VISUALIZER)
    add_subdirectory(plotter)
    add_subdirectory(localization_viz)
endif()

if (BUILD_GNC AND BUILD_VISION)
    add_subdirectory(octomap)
    add_subdirectory(octomap_viz)
endif()