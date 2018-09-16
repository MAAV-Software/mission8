find_path(G2O_INCLUDE_DIR g2o/config.h
    PATHS ${PROJECT_SOURCE_DIR}/thirdparty/g2o
)

set(G2O_INCLUDE_DIRS ${G2O_INCLUDE_DIR})

mark_as_advanced(G2O_INCLUDE_DIR)
