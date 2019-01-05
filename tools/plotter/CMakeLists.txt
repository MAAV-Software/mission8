if(NOT BUILD_PLOTTER)
    return()
endif()

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed
set(CMAKE_AUTOMOC ON)
# Create code from a list of Qt designer ui files
set(CMAKE_AUTOUIC ON)

# Find the QtWidgets library
find_package(Qt5 COMPONENTS Widgets PrintSupport REQUIRED)
find_package(OpenGL REQUIRED)

#enable OpenGl
add_definitions(-DQCUSTOMPLOT_USE_OPENGL)

set(EXECUTABLE maav-plotter)

set(PLOTTER_SRC
    qcustomplot.cpp
    mainwindow.ui
    mainwindow.cpp
    plotter.cpp
)

# Tell CMake to create the plotter executable
add_executable(${EXECUTABLE} ${PLOTTER_SRC})

target_include_directories(${EXECUTABLE} PUBLIC
    plotter
    Qt5::Widgets
    Qt5::PrintSupport
    ${ZCM_INCLUDE_DIRS}
    ${SW_INCLUDE_DIR}
    ${OPENGL_INCLUDE_DIRS}
)

# Use the Widgets module from Qt 5
target_link_libraries(${EXECUTABLE}
    Qt5::Widgets
    Qt5::PrintSupport
    maav-utils
    maav-msg
    ${ZCM_LIBRARIES}
    ${YAMLCPP_LIBRARY}
    ${OPENGL_LIBRARIES}
    stdc++fs
)