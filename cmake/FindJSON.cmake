find_path(JSON_INCLUDE_DIR json.h
	PATHS
	/usr/include/json-c
	/usr/local/include/json-c)

find_library(JSON_LIBRARY
	NAMES json-c
	PATHS ${LIBRARY_PATHS})

mark_as_advanced(JSON_INCLUDE_DIR JSON_LIBRARIES)