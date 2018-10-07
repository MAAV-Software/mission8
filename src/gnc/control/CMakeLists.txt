 add_library(maav-control SHARED
     pid.cpp
 )

 target_include_directories(maav-control PUBLIC ${SW_INCLUDE_DIR}
 )

 target_link_libraries(maav-control
     maav-state
 )
