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
set(Qt5_DIR "${PROJECT_SOURCE_DIR}/thirdparty/Qt5/5.12.1/gcc_64/lib/cmake/Qt5")
find_package(Qt5 5.12.1 REQUIRED COMPONENTS Widgets Core DataVisualization OpenGL PrintSupport
    PATHS ${SOFTWARE_SOURCE_DIR}/thirdparty
    NO_DEFAULT_PATH
    NO_CMAKE_FIND_ROOT_PATH
    QUIET)
find_package(OpenGL REQUIRED)

#enable OpenGl
add_definitions(-DQCUSTOMPLOT_USE_OPENGL)

set(EXECUTABLE maav-plotter)

set(PLOTTER_SRC
    AbstractData.cpp
    DataDict.cpp
    DataDict_generated.cpp
    GaussianData.cpp
    LinePlotWindow.ui
    LinePlotWindow.cpp
    ListWindow.ui
    ListWindow.cpp
    ListWindow_generated.cpp
    Plotter.cpp
    qcustomplot.cpp 
    ZcmConversion.cpp
    ZcmLoop.cpp
    ZcmLoop_generated.cpp
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

# Tell CMake to create the plotter executable
add_executable(${EXECUTABLE} ${PLOTTER_SRC})

target_include_directories(${EXECUTABLE} PUBLIC
    plotter
    ${ZCM_INCLUDE_DIRS}
    ${SW_INCLUDE_DIR}
    ${OPENGL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

# Use the Widgets module from Qt 5
target_link_libraries(${EXECUTABLE}
    Qt5::Core
    Qt5::Widgets
    Qt5::DataVisualization
    Qt5::PrintSupport
    maav-utils
    maav-msg
    ${ZCM_LIBRARIES}
    ${YAMLCPP_LIBRARY}
    ${OPENGL_LIBRARIES}
    stdc++fs
)