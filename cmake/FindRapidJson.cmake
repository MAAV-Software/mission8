find_path(RAPID_JSON_INCLUDE_DIR rapidjson/allocators.h
    PATH_SUFFIXES rapidjson rapidjson/include
    /usr/local/include
)

mark_as_advanced(RAPID_JSON_INCLUDE_DIR)