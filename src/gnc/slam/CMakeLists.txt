add_library(maav-slam SHARED
    Converter.cc
    Frame.cc
    FrameDrawer.cc
    Initializer.cc
    KeyFrame.cc
    KeyFrameDatabase.cc
    LocalMapping.cc
    LoopClosing.cc
    Map.cc
    MapDrawer.cc
    MapPoint.cc
    Optimizer.cc
    ORBextractor.cc
    ORBmatcher.cc
    PnPsolver.cc
    Sim3Solver.cc
    System.cc
    Tracking.cc
    Viewer.cc
)

target_include_directories(maav-slam PUBLIC
    ${GNC_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${Pangolin_INCLUDE_DIRS}
    ${DBOW_INCLUDE_DIR}
    ${G2O_INCLUDE_DIR}
)

target_link_libraries(maav-slam
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
    ${Pangolin_LIBRARIES}
    DBoW2
    g2o
)

