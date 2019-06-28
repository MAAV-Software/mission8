find_path(RAPID_JSON_INCLUDE_DIR rapidjson/allocators.h
    PATH_SUFFIXES rapidjson rapidjson/include
    PATHS ${SOFTWARE_SOURCE_DIR}/thirdparty
    NO_DEFAULT_PATH
    )

mark_as_advanced(RAPID_JSON_INCLUDE_DIR)